#include "common.hpp"
#include "network.hpp"
#include "app.hpp"

namespace network {

bool Network::pathfind (Segment* start, Segment* target, Agent* path) {
	// use dijstra

	struct Comparer {
		bool operator () (Node* l, Node* r) {
			// true: l < r
			// but since priority_queue only lets us get
			// max element rather than min flip everything around
			return l->_cost > r->_cost;
		}
	};
	std::priority_queue<Node*, std::vector<Node*>, Comparer> unvisited;
		
	// queue all nodes
	for (auto& node : nodes) {
		node->_cost = INF;
		node->_visited = false;
		node->_pred = nullptr;
	}

	// TODO: could actually start only on the lane facing the target building
	//  (ie. don't allow crossing the road middle)
	{ // handle the two start nodes
		start->node_a->_cost = start->lane_length / 0.5f; // pretend start point is at center of start segment for now
		start->node_b->_cost = start->lane_length / 0.5f;
	}

	unvisited.push(start->node_a);
	unvisited.push(start->node_b);

	while (!unvisited.empty()) {
		// visit node with min distance
		Node* cur_node = unvisited.top();
		unvisited.pop();
			
		if (cur_node->_visited) continue;
		cur_node->_visited = true;

		// Just an optimization
		//if (cur_node == target->a || cur_node == target->b)
		//	break; // shortest path found (either a or b works becase shortest one will always be visited first)
		//	       // -> not quite true due to the possible difference in additional distance along the final segment! -> if either node found just check again that both are visited

		// update neighbours with new minimum distances
		for (auto& seg : cur_node->segments) {
			int dir = cur_node == seg->node_a ? 0 : 1;
			Node* other_node = dir ? seg->node_a : seg->node_b;

			for (int i=0; i<(int)seg->layout->lanes.size(); ++i) {
				auto& lane = seg->layout->lanes[i];

				if (lane.direction == dir) { // TODO: could cache list to avoid this check and iterate half as many lanes

					float new_cost = cur_node->_cost + seg->lane_length + seg->node_a->radius + seg->node_b->radius; // TODO: ??
					if (new_cost < other_node->_cost) {
						other_node->_pred      = cur_node;
						other_node->_pred_seg  = { seg, i };
						other_node->_cost      = new_cost;

						unvisited.push(other_node); // push updated neighbour (duplicate)
					}
				}
			}
		}
	}

	//// make path out of dijkstra graph
	if (target->node_a->_pred == nullptr && target->node_b->_pred == nullptr)
		return false; // no path found
		
	// additional distances from a and b of the target segment
	float dist_from_a = 0.5f;
	float dist_from_b = 0.5f;

	float a_cost = target->node_a->_cost + dist_from_a;
	float b_cost = target->node_b->_cost + dist_from_b;
	// choose end node that end up fastest
	Node* end_node = a_cost < b_cost ? target->node_a : target->node_b;

	assert(end_node->_cost < INF);

	std::vector<Node*> tmp_nodes;

	Node* cur = end_node;
	while (cur) {
		tmp_nodes.push_back(cur);
		cur = cur->_pred;
	}
	assert(tmp_nodes.size() >= 1);

	Node* start_node = tmp_nodes.back();

	// first node
	path->nodes.push_back(start_node);

	// start segment
	// if start node is segment.a then we go from b->a and thus need to start on a reverse lane
	int start_lane = start_node == start->node_b ? 0 : (int)start->layout->lanes.size()-1;
	path->segments.push_back({ start, start_lane });

	// nodes
	for (int i=(int)tmp_nodes.size()-2; i>=0; --i) {
		assert(tmp_nodes[i]->_pred_seg.seg);
		// segment of predecessor to path node
		path->segments.push_back( tmp_nodes[i]->_pred_seg );
		// path node
		path->nodes.push_back(tmp_nodes[i]);
	}

	// end segment
	// if end node is segment.a then we go from a->b and thus need to start on a forward lane
	int end_lane = end_node == target->node_a ? 0 : (int)target->layout->lanes.size()-1;
	path->segments.push_back({ target, end_lane });

	return true;
}

enum State { EXIT_BUILDING, ENTER_BUILDING, SEGMENT, NODE };
struct Bezier3 {
	float3 a, b, c;

	void dbg_draw (View3D const& view, int res, lrgba col, float t0=0, float t1=1) const {
		float2 prev = bezier3(t0, (float2)a, (float2)b, (float2)c).pos;
		for (int i=0; i<res; ++i) {
			float t = lerp(t0, t1, (float)(i+1) / res);

			auto bez = bezier3(t, (float2)a, (float2)b, (float2)c);
			
			if (i < res-1) {
				g_dbgdraw.line(float3(prev, a.z), float3(bez.pos, a.z), col);
			}
			else {
				g_dbgdraw.arrow(view, float3(prev, a.z), float3(bez.pos - prev, 0), 1, col);
			}

			prev = bez.pos;
		}
	}
};


inline Bezier3 calc_curve (Line const& l0, Line const& l1) {
	float2 point;
	if (!line_line_intersect((float2)l0.a, (float2)l0.b, (float2)l1.a, (float2)l1.b, &point))
		point = (l0.b+l1.a)*0.5f;
		//point = (l0.b+l1.a)*0.5f + 0.5f * distance(l0.a, l1.b) * normalizesafe(l0.b - l0.a);
					
	return { l0.b, float3(point, l0.a.z), l1.a };
}

struct AgentState {
	State state;
	float end_t = 1.0f;
	float next_start_t = 0.0f;
	Bezier3 bezier;

	network::AgentList* cur_agents = nullptr;
	network::AgentList* next_agents = nullptr;

	network::Node* cur_node = nullptr;
	network::SegLane* seg_before_node = nullptr;
	network::SegLane* seg_after_node = nullptr;
};
AgentState _FORCEINLINE get_agent_state (Agent* agent, int idx) {
	AgentState s;

	int num_nodes  = (int)agent->nodes.size();
	int num_seg    = (int)agent->segments.size();
	int num_moves = num_nodes + num_seg + 2;

	assert(num_nodes >= 1);
	assert(num_nodes + 1 == num_seg);
	assert(idx < num_moves);

	if (idx == 0) {
		auto s_seg = agent->segments.front();
		auto s_lane = s_seg.clac_lane_info();
		float3 s0 = agent->start->pos;
		float3 s1 = (s_lane.a + s_lane.b) * 0.5f;

		s.state = EXIT_BUILDING;
		s.next_start_t = 0.5f;

		s.next_agents = &s_seg.seg->agents.lanes[s_seg.lane];

		s.bezier = { s0, (s0+s1)*0.5f, s1 };
	}
	else if (idx == num_moves-1) {
		auto e_seg = agent->segments.back();
		auto e_lane = e_seg.clac_lane_info();
		float3 e0 = (e_lane.a + e_lane.b) * 0.5f;
		float3 e1 = agent->end->pos;

		s.state = ENTER_BUILDING;

		s.bezier = { e0, (e0+e1)*0.5f, e1 };
	}
	else {
		int i = idx - 1;
				
		bool last_seg = i/2+1 >= num_seg;
		assert(i/2 < num_nodes == !last_seg);

		auto* seg = &agent->segments[i/2];
		auto l = seg->clac_lane_info();

		auto* node = !last_seg ? agent->nodes[i/2] : nullptr;

		auto* seg2 = !last_seg ? &agent->segments[i/2+1] : nullptr;
		auto l2 = seg2 ? seg2->clac_lane_info() : Line{0,0};

		s.cur_node = node;
		s.seg_before_node = seg;
		s.seg_after_node = seg2;

		if (node) assert(contains(node->in_lanes, *s.seg_before_node));
		if (node) assert(contains(node->out_lanes, *s.seg_after_node));

		if (i % 2 == 0) {
			assert(seg);

			if (s.seg_before_node && agent->idx == idx)
				assert(contains(s.seg_before_node->seg->agents.lanes[s.seg_before_node->lane].list, agent));

			s.state = SEGMENT;

			s.cur_agents = &seg->seg->agents.lanes[seg->lane];
			s.next_agents = node ? &node->agents.free : nullptr;


			// handle enter building
			if (last_seg)
				s.end_t = 0.5f;

			s.bezier = { l.a, (l.a+l.b)*0.5f, l.b };
		}
		else {
			assert(seg && node && seg2);

			if (agent->idx == idx)
				assert(contains(s.cur_node->agents.free.list, agent));

			s.state = NODE;

			s.cur_agents = &node->agents.free;
			s.next_agents = &seg2->seg->agents.lanes[seg2->lane];

			float2 point;
			if (!line_line_intersect((float2)l.a, (float2)l.b, (float2)l2.a, (float2)l2.b, &point))
				point = (l.b+l2.a)*0.5f;
				//point = (l0.b+l1.a)*0.5f + 0.5f * distance(l0.a, l1.b) * normalizesafe(l0.b - l0.a);
					
			s.bezier = calc_curve(l, l2);
		}
	}

	return s;
}
	
float brake_for_dist (float obstacle_dist) {
	return clamp(map(obstacle_dist, 0.0f, 8.0f), 0.0f, 1.0f);
}

void debug_node (App& app, Node* node) {
	if (!node) return;

	g_dbgdraw.wire_circle(node->pos, node->radius, lrgba(1,1,0,1));

	{
		int i=0;
		for (auto& agent : node->agents.free.list) {
			g_dbgdraw.wire_circle(agent->cit->_pos, CAR_SIZE*0.5f, lrgba(1,0,0.5f,1));

			g_dbgdraw.text.draw_text(prints("%d", i++),
				30, 1, g_dbgdraw.text.map_text(agent->cit->_pos, app.view));
		}
	}

	int col_i = 0;
	//node->for_ingoing_lanes([&] (SegLane lane_in) {
	//	auto l0 = lane_in.seg->clac_lane_info(lane_in.lane);
	//			
	//	lrgba col = g_dbgdraw.COLS[col_i++ % ARRLEN(g_dbgdraw.COLS)];
	//			
	//	node->for_outgoing_lanes([&] (SegLane lane_out) {
	//		auto l1 = lane_out.seg->clac_lane_info(lane_out.lane);
	//		calc_curve(l0, l1).dbg_draw(10, col);
	//	});
	//});

	//int col_i = 0;
	//node->for_ingoing_lanes([&] (SegLane lane_in) {
	//	auto l0 = lane_in.seg->clac_lane_info(lane_in.lane);
	//			
	//	lrgba col = g_dbgdraw.COLS[col_i++ % ARRLEN(g_dbgdraw.COLS)];
	//			
	//	node->for_outgoing_lanes([&] (SegLane lane_out) {
	//		auto l1 = lane_out.seg->clac_lane_info(lane_out.lane);
	//			
	//		float2 point;
	//		if (!line_line_intersect((float2)l0.a, (float2)l0.b, (float2)l1.a, (float2)l1.b, &point))
	//			point = (l0.b+l1.a)*0.5f;
	//		
	//		Bezier3 bezier = { l0.b, float3(point, l0.a.z), l1.a };
	//		bezier.dbg_draw(10, col);
	//	});
	//});
}

void debug_collision_points (Agent* agent, lrgba col) {
	
	float3 prev = agent->cit->_pos;
	for (int i=0; i<COLLISION_STEPS; ++i) {
		float3 pos = float3(agent->cit->path->collision_points[i], agent->cit->_pos.z);
		
		dbg_draw_boxy_line(prev, pos, LANE_COLLISION_R, col);
		prev = pos;
	}
}
void debug_citizen (App& app, Citizen* cit) {
	if (!cit || !cit->path) return;

	float start_t = cit->path->cur_t;
	for (int i=cit->path->idx; ; ++i) {
		auto s = get_agent_state(cit->path.get(), i);

		s.bezier.dbg_draw(app.view, 5, lrgba(1,1,0,1), start_t, s.end_t); 

		start_t = s.next_start_t;
		if (s.state == ENTER_BUILDING) break;
	}
	
	debug_collision_points(cit->path.get(), lrgba(1,0,1,1));
}

void update_collision_points (Agent* agent) {
	int idx = agent->idx;
	auto s = get_agent_state(agent, idx);

	float t = agent->cur_t;

	AABB2 bounds;

	for (int i=0; i<COLLISION_STEPS; ++i) {
		t += 0.25f;
		if (t > s.end_t) {
			if (s.state == ENTER_BUILDING) break; // TODO: either store i or fill collision_points with sensible points
			t = t - s.end_t + s.next_start_t;
			s = get_agent_state(agent, ++idx);
		}

		auto bez = bezier3(t, s.bezier.a, s.bezier.b, s.bezier.c);
		agent->collision_points[i] = bez.pos;

		bounds.add(bez.pos);
	}

	bounds.lo -= LANE_COLLISION_R * SQRT_2; // account for collision radius with boxy radius
	bounds.hi += LANE_COLLISION_R * SQRT_2;
	agent->collision_points_bounds = bounds;
}

bool check_conflict (Agent* a, Agent* b, float* out_a_dist, float* out_b_dist) {
	assert(a != b);

	if (!a->collision_points_bounds.overlap( b->collision_points_bounds ))
		return false;

	float min_a_dist = INF;
	float min_b_dist = INF;

	float a_dist = 0;
	float2 prev_pa = (float2)a->cit->_pos;
	for (auto& pa : a->collision_points) {
		float2 da = pa - prev_pa;
		float a_len = length(da);

		float b_dist = 0;
		float2 prev_pb = (float2)b->cit->_pos;
		for (auto& pb : b->collision_points) {
			float2 db = pb - prev_pb;
			float b_len = length(db);

			float u, v;
			bool hit_a = ray_box_intersection(prev_pa, da, prev_pb, db, b_len, LANE_COLLISION_R*2, &u);
			bool hit_b = ray_box_intersection(prev_pb, db, prev_pa, da, a_len, LANE_COLLISION_R*2, &v);
			if (hit_a) min_a_dist = min(min_a_dist, a_dist + u*a_len);
			if (hit_b) min_b_dist = min(min_b_dist, b_dist + v*b_len);
			
			b_dist += b_len;
			prev_pb = pb;
		}

		a_dist += a_len;
		prev_pa = pa;
	}
	
	*out_a_dist = min_a_dist;
	*out_b_dist = min_b_dist;

	return min_a_dist < INF;
}

void Network::simulate (App& app) {
	ZoneScoped;

	pathing_count = 0;

	auto do_pathfind = [&] (Citizen* cit) {
		auto* cur_target = app.entities.buildings[ app.test_rand.uniformi(0, (int)app.entities.buildings.size()) ].get();
	
		cit->_pos = cit->building->pos;
		cit->_rot = 0;

		assert(cit->building->connected_segment);
		if (cit->building->connected_segment) {
			ZoneScopedN("pathfind");

			pathing_count++;

			auto path = std::make_unique<network::Agent>();
			path->cit = cit;
			path->start = cit->building;
			path->end   = cur_target;
			bool valid = pathfind(cit->building->connected_segment, cur_target->connected_segment, path.get());
			if (valid) {
				cit->building = nullptr;
				cit->path = std::move(path);
				// keep _pos & _rot
			}
		}
	};
	
	// to avoid debugging overlays only showing while not paused, only skip moving the car when paused, later actually skip sim steps
	//if (!app.sim_paused) {
		auto update_segment = [&] (Segment* seg) {
			for (auto& lane : seg->agents.lanes) {
				// brake for car in front
				for (int i=1; i<(int)lane.list.size(); ++i) {
					Agent* prev = lane.list[i-1];
					Agent* cur = lane.list[i];
					
					float dist = distance((float2)cur->cit->_pos, (float2)prev->cit->_pos);
					dist -= CAR_SIZE + 1;
					cur->brake = min(cur->brake, brake_for_dist(dist));
				}
			}
		};
		
		auto update_node = [&] (Node* node) {
			//struct Connection {
			//	Segment* seg_a;
			//	Segment* seg_b;
			//	int      lane_a;
			//	int      lane_b;
			//
			//	Connection (SegLane const& a, SegLane const& b): seg_a{a.seg}, seg_b{b.seg}, lane_a{a.lane}, lane_b{b.lane} {}
			//
			//	bool operator== (Connection const& other) const {
			//		return memcmp(this, &other, sizeof(*this)) == 0;
			//	}
			//	bool operator!= (Connection const& other) const {
			//		return !(*this == other);
			//	}
			//};
			VALUE_HASHER(SegLane, t.seg, t.lane);
			Hashmap<SegLane, float, SegLaneHasher> avail_space;

			auto dbg_avail_space = [&] (SegLane const& lane_out, Agent* a) {
				if (app.selection.get<Node*>() != node) return;

				auto li = lane_out.clac_lane_info();
				
				auto pos = lerp(li.a, li.b, avail_space[lane_out] / lane_out.seg->lane_length);
				g_dbgdraw.point(pos, 1, lrgba(1,0,1,1));
				g_dbgdraw.point(pos, 1, lrgba(1,0,1,1));
			};

			//
			int col_i = 0;
			for (auto& lane_out : node->out_lanes) {
				avail_space.add(lane_out, lane_out.seg->lane_length);
				
				for (auto* a : lane_out.seg->agents.lanes[lane_out.lane].list) {
					dbg_avail_space(lane_out, a);

					avail_space[lane_out] -= CAR_SIZE + 1; // Use real car length for specific car here
				}
			}

			// TODO: reserve space only if car is not waiting for other cars, probably by doing it in the loops below
			
			auto vehicle_conflict = [&] (Agent* a, Agent* b) {
				assert(a != b);
				if (a->blocked || b->blocked)
					return;

				float a_dist, b_dist;
				if (!check_conflict(a, b, &a_dist, &b_dist))
					return;

				auto sa = get_agent_state(a, a->idx);
				auto sb = get_agent_state(b, b->idx);

				auto a_lane = sa.seg_before_node->clac_lane_info();
				auto b_lane = sb.seg_before_node->clac_lane_info();

				float2 a_dir = normalizesafe(a_lane.b - a_lane.a);
				float2 b_dir = normalizesafe(b_lane.b - b_lane.a);
				float ab_cross = cross(a_dir, b_dir);
				bool a_has_prio = ab_cross < -0.2f;
				bool b_has_prio = ab_cross > 0.2f;

				float e_a_dist = a_dist;
				float e_b_dist = b_dist;

				if (     a_has_prio) e_a_dist *= 0.33f;
				else if (b_has_prio) e_b_dist *= 0.33f;

				Agent* waiting     = e_a_dist > e_b_dist ? a : b;
				float waiting_dist = e_a_dist > e_b_dist ? a_dist : b_dist;

				if (a->cit == app.selection.get<Citizen*>()) {
					auto c = a == waiting ? lrgba(1,0,0,1) : lrgba(0.1f,0,0,1);
					debug_collision_points(b, c);

					float2 dir = normalizesafe(a->collision_points[0] - (float2)a->cit->_pos);
					
					g_dbgdraw.point(a->cit->_pos + float3(dir,0) * a_dist, 2, c);
					if (a == waiting)
						g_dbgdraw.wire_circle(b->cit->_pos, CAR_SIZE/2, lrgba(1,0,0,1));
				}
				if (b->cit == app.selection.get<Citizen*>()) {
					auto c = b == waiting ? lrgba(1,0,0,1) : lrgba(0.1f,0,0,1);
					debug_collision_points(a, c);

					float2 dir = normalizesafe(b->collision_points[0] - (float2)b->cit->_pos);
					
					g_dbgdraw.point(b->cit->_pos + float3(dir,0) * b_dist, 2, c);
					if (b == waiting)
						g_dbgdraw.wire_circle(a->cit->_pos, CAR_SIZE/2, lrgba(1,0,0,1));
				}
				
				float dist = waiting_dist;

				auto s = get_agent_state(waiting, waiting->idx);
				if (s.state == SEGMENT) {
					float2 lane_end = (float2)s.seg_before_node->clac_lane_info().b;
					
					dist = distance((float2)waiting->cit->_pos, lane_end) - CAR_SIZE*0.5f;
				}
				waiting->brake = min(waiting->brake, brake_for_dist(dist));
				waiting->blocked = true;
			};

			for (int i=0; i<(int)node->agents.free.list.size(); ++i) {
				auto* a = node->agents.free.list[i];
				
				// count space in target lane
				auto s = get_agent_state(a, a->idx);
				dbg_avail_space(*s.seg_after_node, a);
				avail_space[*s.seg_after_node] -= CAR_SIZE + 1; // Use real car length for specific car here

				// process remaining node agents
				for (int j=i+1; j<(int)node->agents.free.list.size(); ++j) {
					vehicle_conflict(a, node->agents.free.list[j]);
				}
				
				// process last agents in outgoing lanes
				for (auto& out_lane : node->out_lanes) {
					auto& agents = out_lane.seg->agents.lanes[out_lane.lane].list;
					if (!agents.empty())
						vehicle_conflict(a, agents.back());
				}

				// process ingoing lanes
				for (auto& in_lane : node->in_lanes) {
					for (auto& b : in_lane.seg->agents.lanes[in_lane.lane].list) {
						vehicle_conflict(a, b);
					}
				}
			}

			for (auto& lane : node->in_lanes) {
				for (auto* agent : lane.seg->agents.lanes[lane.lane].list) {
					auto s = get_agent_state(agent, agent->idx);
					if (!s.seg_after_node) continue;
					if (agent->blocked) break;

					if (avail_space[*s.seg_after_node] < CAR_SIZE) {
						float2 lane_end = (float2)lane.clac_lane_info().b;
							
						float dist = distance((float2)agent->cit->_pos, lane_end) - CAR_SIZE*0.5f;
						agent->brake = min(agent->brake, brake_for_dist(dist));
						agent->blocked = true;
					}
					else {
						dbg_avail_space(*s.seg_after_node, agent);
						avail_space[*s.seg_after_node] -= CAR_SIZE + 1;
					}
				}
			}
			
			// process ingoing lanes
			for (int i=0; i<(int)node->in_lanes.size()-1; ++i) {
				auto& lane = node->in_lanes[i];
				for (auto& a : lane.seg->agents.lanes[lane.lane].list) {
					if (a->blocked) break;
				
					// process remaining ingoing lanes
					for (int j=i+1; j<(int)node->in_lanes.size(); ++j) {
						auto& other_lane = node->in_lanes[j];
						for (auto& b : other_lane.seg->agents.lanes[other_lane.lane].list) {
							if (b->blocked) break;

							vehicle_conflict(a, b);
						}
					}
				}
			}
		};
		
		auto update_vehile1 = [&] (Agent* agent) {

		};
		auto update_vehicle = [&] (Agent* agent) {
			assert(agent->cur_t < 1.0f);

			auto s = get_agent_state(agent, agent->idx);

			if (!app.sim_paused) { // move
				auto bez = bezier3(agent->cur_t, (float2)s.bezier.a, (float2)s.bezier.b, (float2)s.bezier.c);
				float bez_speed = length(bez.vel); // delta pos / bezier t

				agent->cit->_pos = float3(bez.pos, s.bezier.a.z);
				agent->cit->_rot = angle2d(bez.vel);

				// (delta pos / delta time)[speed] * time[dt] / (delta pos / bezier delta t)[bez_speed]
				// -> delta t
				agent->cur_t += top_speed * app.input.dt * agent->brake / bez_speed;
			}
			
			if (agent->cur_t >= s.end_t) {
				agent->idx++;
				agent->cur_t = s.next_start_t;

				if (s.cur_agents)  s.cur_agents ->remove(agent);
				if (s.next_agents) s.next_agents->add(agent);

				if (s.state == ENTER_BUILDING) {
					// end path
					agent->cit->building = agent->end;
					agent->cit->path = nullptr;
				}
			}
		};
		
		{
			ZoneScopedN("init pass");
			for (auto& cit : app.entities.citizens) {
				if (cit->building) continue;
				cit->path->brake = 1;
				cit->path->blocked = false;
				update_collision_points(cit->path.get());
			}
		}
		
		{
			ZoneScopedN("update segments");
			for (auto& seg : segments) {
				update_segment(seg.get());
			}
		}
		{
			ZoneScopedN("update nodes");
			for (auto& node : nodes) {
				update_node(node.get());
			}
		}

		{
			ZoneScopedN("car pass");
			for (auto& cit : app.entities.citizens) {
				if (!cit->building) {
					update_vehile1(cit->path.get());
				}
			}
		}
		
		{
			ZoneScopedN("final pass");
			for (auto& cit : app.entities.citizens) {
				if (cit->building) {
					do_pathfind(cit.get());
				}
				else {
					update_vehicle(cit->path.get());
				}
			}
		}
	//}

	debug_node(app, app.selection.get<Node*>());
	debug_citizen(app, app.selection.get<Citizen*>());

	static RunningAverage pathings_avg(30);
	pathings_avg.push((float)pathing_count);
	float min, max;
	float avg = pathings_avg.calc_avg(&min, &max);
	ImGui::Text("pathing_count: avg %3.1f min: %3.1f max: %3.1f", avg, min, max);
}

} // namespace network
