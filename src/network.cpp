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
AgentState get_agent_state (Agent* agent, int idx) {
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

		if (i % 2 == 0) {
			assert(seg);

			s.state = SEGMENT;

			s.cur_agents = &seg->seg->agents.lanes[seg->lane];
			s.next_agents = node ? &node->agents.free : nullptr;

			s.cur_node = node;
			s.seg_before_node = seg;
			s.seg_after_node = seg2;

			// handle enter building
			if (last_seg)
				s.end_t = 0.5f;

			s.bezier = { l.a, (l.a+l.b)*0.5f, l.b };
		}
		else {
			assert(seg && node && seg2);

			s.state = NODE;

			s.cur_agents = &node->agents.free;
			s.next_agents = &seg2->seg->agents.lanes[seg2->lane];

			s.cur_node = node;
			s.seg_before_node = seg;
			s.seg_after_node = seg2;

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

	if (ImGui::TreeNodeEx("dbg_conflicts")) {

		struct Conn { SegLane in, out; };
		auto idx2conn = [] (Node* node, int idx) {
			return Conn{
				node->in_lanes [idx / (int)node->out_lanes.size()],
				node->out_lanes[idx % (int)node->out_lanes.size()],
			};
		};

		int num_conns = node->num_conns();
		
		static int view_conn = 1;
		static int other_conn = 8;
		ImGui::SliderInt("view_conn", &view_conn, 0, num_conns-1);
		ImGui::SliderInt("other_conn", &other_conn, 0, num_conns-1);
		view_conn = clamp(view_conn, 0, num_conns-1);
		other_conn = clamp(other_conn, 0, num_conns-1);

		auto a = idx2conn(node, view_conn);
		auto b = idx2conn(node, other_conn);

		auto check_paths = [] (Conn& a, Conn& b, float width) {
			auto bez_a = calc_curve(a.in.clac_lane_info(), a.out.clac_lane_info());
			auto bez_b = calc_curve(b.in.clac_lane_info(), b.out.clac_lane_info());

			struct Point { float2 pos; float t; };
			std::vector<Point> points_a;
			std::vector<Point> points_b;

			for (float t = 0;;) {
				auto val = bezier3(t, (float2)bez_a.a, (float2)bez_a.b, (float2)bez_a.c);
				points_a.push_back({ val.pos, t });

				g_dbgdraw.wire_circle(float3(val.pos, bez_a.a.z), width/2, lrgba(1,0,1));

				t += width*0.5f / length(val.vel);
				if (t >= 1.0f) break;
			}
			for (float t = 0;;) {
				auto val = bezier3(t, (float2)bez_b.a, (float2)bez_b.b, (float2)bez_b.c);
				points_b.push_back({ val.pos, t });

				g_dbgdraw.wire_circle(float3(val.pos, bez_a.a.z), width/2, lrgba(0,1,1));
				
				t += width*0.5f / length(val.vel);
				if (t >= 1.0f) break;
			}
			
			//float a0=1, a1=0;
			//float b0=1, b1=0;

			for (auto& a : points_a) {
				for (auto& b : points_b) {
					float dist_sqr = length_sqr(b.pos - a.pos);
					if (dist_sqr < width*width)
						return true;
				}
			}

			return false;
		};
		bool conflict = check_paths(a, b, node->segments[0]->layout->lanes[0].width - 0.1f); // offset to avoid collisions due to numerics

		calc_curve(a.in.clac_lane_info(), a.out.clac_lane_info()).dbg_draw(app.view, 10, lrgba(1,1,0,1));
		calc_curve(b.in.clac_lane_info(), b.out.clac_lane_info()).dbg_draw(app.view, 10,
			conflict ? lrgba(1,0,0,1) : lrgba(0,1,0,1));

		ImGui::TreePop();
	};


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

	col_i = 0;
	for (auto& lane_out : node->out_lanes) {
		lrgba col = g_dbgdraw.COLS[col_i++ % ARRLEN(g_dbgdraw.COLS)];
					
		float avail_space = lane_out.seg->lane_length;
		for (auto* a : lane_out.seg->agents.lanes[lane_out.lane].list) {
			auto p = lane_out.clac_lane_info();
			auto pos = lerp(p.a, p.b, avail_space / lane_out.seg->lane_length);
			g_dbgdraw.point(pos, 1, col);
	
			avail_space -= CAR_SIZE + 1; // Use real car length for specific car here
		}
		for (auto* a : node->agents.free.list) {
			auto s = get_agent_state(a, a->idx);
			if (*s.seg_after_node != lane_out) continue;
	
			auto p = lane_out.clac_lane_info();
			auto pos = lerp(p.a, p.b, avail_space / lane_out.seg->lane_length);
			g_dbgdraw.point(pos, 1, col);
	
			avail_space -= CAR_SIZE + 1; // Use real car length for specific car here
		}
	
		ImGui::TextColored(ImVec4(col.x, col.y, col.z, col.w), "%.3f / %.3f", avail_space, lane_out.seg->lane_length);
			
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
}

bool check_conflict (SegLane& a0, SegLane& a1, SegLane& b0, SegLane& b1) {
	if (a0 == b0 || a1 == b1)
		return true;

	float width = 3.4f; // TODO: at least make a global

	auto bez_a = calc_curve(a0.clac_lane_info(), a1.clac_lane_info());
	auto bez_b = calc_curve(b0.clac_lane_info(), b1.clac_lane_info());

	struct Point { float2 pos; float t; };
	std::vector<Point> points_a;
	std::vector<Point> points_b;

	for (float t = 0;;) {
		auto val = bezier3(t, (float2)bez_a.a, (float2)bez_a.b, (float2)bez_a.c);
		points_a.push_back({ val.pos, t });

		t += width*0.5f / length(val.vel);
		if (t >= 1.0f) break;
	}
	for (float t = 0;;) {
		auto val = bezier3(t, (float2)bez_b.a, (float2)bez_b.b, (float2)bez_b.c);
		points_b.push_back({ val.pos, t });

		t += width*0.5f / length(val.vel);
		if (t >= 1.0f) break;
	}
			
	//float a0=1, a1=0;
	//float b0=1, b1=0;

	for (auto& a : points_a) {
		for (auto& b : points_b) {
			float dist_sqr = length_sqr(b.pos - a.pos);
			if (dist_sqr < width*width)
				return true;
		}
	}

	return false;
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
	
	if (!app.sim_paused) {
		auto update_node = [&] (Node* node) {
			for (auto& lane_out : node->out_lanes) {
				float avail_space = lane_out.seg->lane_length;
				for (auto* a : lane_out.seg->agents.lanes[lane_out.lane].list) {
					avail_space -= CAR_SIZE + 1; // Use real car length for specific car here
				}
				for (auto* a : node->agents.free.list) {
					auto s = get_agent_state(a, a->idx);
					if (*s.seg_after_node != lane_out) continue;

					avail_space -= CAR_SIZE + 1; // Use real car length for specific car here
				}

				for (auto& lane_in : node->in_lanes) {
					auto& agents = lane_in.seg->agents.lanes[lane_in.lane];
						
					float2 lane_end = (float2)lane_in.clac_lane_info().b;
					
					for (auto* agent : agents.list) {
						auto s = get_agent_state(agent, agent->idx);
						if (!s.seg_after_node || *s.seg_after_node != lane_out) continue;

						if (avail_space < CAR_SIZE) {
							float dist = distance((float2)agent->cit->_pos, lane_end) - CAR_SIZE*0.5f;
							agent->brake = min(agent->brake, brake_for_dist(dist));

							avail_space -= CAR_SIZE + 1;
						}
					}
				}
			}

		};
		
		auto update_vehile1 = [&] (Agent* agent) {
			auto s = get_agent_state(agent, agent->idx);

			float min_dist = INF;
		
			float2 forw = rotate2(agent->cit->_rot) * float2(1,0);
			float2 right = rotate90(-forw);
			float cone_dot = cos(cone);
			
			auto check = [&] (Agent* other) {
				assert(agent != other);

				float2 offs = other->cit->_pos - agent->cit->_pos;
				//float2 dir = normalizesafe(offs);
			
				float y = dot(forw, offs);
				float x = dot(right, offs);

				//if (y > cone_dot) { // in cone
				if (y >= 0.0f) { // in cone
					float dist = length(offs);
					if (x > 1.0f)
						dist *= 2.0f;
					min_dist = min(min_dist, dist);
				}
			};
		
			// checks car against all higher priority cars (cars that came first)
			if (s.next_agents) {
				for (auto& a : s.next_agents->list) {
					assert(a != agent); // cant be in this list
					
					if (s.state == SEGMENT) {
						auto as = get_agent_state(a, a->idx);
						assert(as.state == NODE);
						if (!check_conflict(*s.seg_before_node, *s.seg_after_node, *as.seg_before_node, *as.seg_after_node))
							continue;
					}

					check(a);
				}
			}
			if (s.cur_agents) {
				for (auto& a : s.cur_agents->list) {
					// don't check ourself or any cars that cam after us
					if (a == agent) break;

					if (s.state == NODE) {
						auto as = get_agent_state(a, a->idx);
						assert(as.state == NODE);
						if (!check_conflict(*s.seg_before_node, *s.seg_after_node, *as.seg_before_node, *as.seg_after_node))
							continue;
					}

					check(a);
				}
			}

			min_dist -= CAR_SIZE + 1;
		
			agent->brake = min(agent->brake, brake_for_dist(min_dist));
		};
		auto update_vehicle = [&] (Agent* agent) {
			assert(agent->cur_t < 1.0f);

			auto s = get_agent_state(agent, agent->idx);

			{ // move
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
	}

	debug_node(app, app.selection.get<Node*>());
	debug_citizen(app, app.selection.get<Citizen*>());

	static RunningAverage pathings_avg(30);
	pathings_avg.push((float)pathing_count);
	float min, max;
	float avg = pathings_avg.calc_avg(&min, &max);
	ImGui::Text("pathing_count: avg %3.1f min: %3.1f max: %3.1f", avg, min, max);
}

} // namespace network
