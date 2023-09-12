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

	void dbg_draw (int count, lrgba col, float t0=0, float t1=1) const {
		float2 prev = bezier3(t0, (float2)a, (float2)b, (float2)c).pos;
		for (int i=0; i<count; ++i) {
			float t = lerp(t0, t1, (float)(i+1) / count);

			auto res = bezier3(t, (float2)a, (float2)b, (float2)c);
						
			g_dbgdraw.line(float3(prev, a.z), float3(res.pos, a.z), col);

			prev = res.pos;
		}
	}
};


inline Bezier3 calc_curve (Segment::Line const& l0, Segment::Line const& l1) {
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
		auto s_lane = s_seg.seg->clac_lane_info(s_seg.lane);
		float3 s0 = agent->start->pos;
		float3 s1 = (s_lane.a + s_lane.b) * 0.5f;

		s.state = EXIT_BUILDING;
		s.next_start_t = 0.5f;

		s.next_agents = &s_seg.seg->agents.lanes[s_seg.lane];

		s.bezier = { s0, (s0+s1)*0.5f, s1 };
	}
	else if (idx == num_moves-1) {
		auto e_seg = agent->segments.back();
		auto e_lane = e_seg.seg->clac_lane_info(e_seg.lane);
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
		auto l = seg->seg->clac_lane_info(seg->lane);

		auto* node = !last_seg ? agent->nodes[i/2] : nullptr;

		auto* seg2 = !last_seg ? &agent->segments[i/2+1] : nullptr;
		auto l2 = seg2 ? seg2->seg->clac_lane_info(seg2->lane) : Segment::Line{0,0};

		if (i % 2 == 0) {
			assert(seg);

			s.state = SEGMENT;

			s.cur_agents = &seg->seg->agents.lanes[seg->lane];
			s.next_agents = node ? &node->agents.free : nullptr;
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

void debug_node (Node* node) {
	if (!node) return;

	g_dbgdraw.wire_circle(node->pos, node->radius, lrgba(1,1,0,1));



	//int col_i = 0;
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

	//col_i = 0;
	//node->for_outgoing_lanes([&] (SegLane lane_out) {
	//	lrgba col = g_dbgdraw.COLS[col_i++ % ARRLEN(g_dbgdraw.COLS)];
	//				
	//	float avail_space = lane_out.seg->lane_length;
	//	for (auto* a : lane_out.seg->agents.lanes[lane_out.lane].list) {
	//		auto p = lane_out.seg->clac_lane_info(lane_out.lane);
	//		auto pos = lerp(p.a, p.b, avail_space / lane_out.seg->lane_length);
	//		g_dbgdraw.point(pos, 1, col);
	//
	//		avail_space -= CAR_SIZE + 1; // Use real car length for specific car here
	//	}
	//	for (auto* a : node->agents.free.list) {
	//		auto s = get_agent_state(a, a->idx);
	//		if (*s.seg_after_node != lane_out) continue;
	//
	//		auto p = lane_out.seg->clac_lane_info(lane_out.lane);
	//		auto pos = lerp(p.a, p.b, avail_space / lane_out.seg->lane_length);
	//		g_dbgdraw.point(pos, 1, col);
	//
	//		avail_space -= CAR_SIZE + 1; // Use real car length for specific car here
	//	}
	//
	//	ImGui::TextColored(ImVec4(col.x, col.y, col.z, col.w), "%.3f / %.3f", avail_space, lane_out.seg->lane_length);
	//		
	//});
}
void debug_citizen (Citizen* cit) {
	if (!cit || !cit->path) return;

	float start_t = cit->path->cur_t;
	for (int i=cit->path->idx; ; ++i) {
		auto s = get_agent_state(cit->path.get(), i);

		s.bezier.dbg_draw(5, lrgba(1,1,0,1), start_t, s.end_t); 

		start_t = s.next_start_t;
		if (s.state == ENTER_BUILDING) break;
	}
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
			node->for_outgoing_lanes([&] (SegLane lane_out) {
				float avail_space = lane_out.seg->lane_length;
				for (auto* a : lane_out.seg->agents.lanes[lane_out.lane].list) {
					avail_space -= CAR_SIZE + 1; // Use real car length for specific car here
				}
				for (auto* a : node->agents.free.list) {
					auto s = get_agent_state(a, a->idx);
					if (*s.seg_after_node != lane_out) continue;

					avail_space -= CAR_SIZE + 1; // Use real car length for specific car here
				}

				node->for_ingoing_lanes([&] (SegLane lane_in) {
					auto& agents = lane_in.seg->agents.lanes[lane_in.lane];
						
					float2 lane_end = (float2)lane_in.seg->clac_lane_info(lane_in.lane).b;
						
					for (auto* agent : agents.list) {
						auto s = get_agent_state(agent, agent->idx);

						if (!s.seg_after_node || *s.seg_after_node != lane_out) continue;

						if (avail_space < CAR_SIZE) {
							float dist = distance((float2)agent->cit->_pos, lane_end) - CAR_SIZE*0.5f;
							agent->_break = min(agent->_break, brake_for_dist(dist));

							avail_space -= CAR_SIZE + 1;
						}
					}
				});
			});
		};
		
		auto update_vehicle = [&] (Agent* agent) {
		
			auto brake_for_cars = [&] (AgentList* agents, AgentList* next_agents, Agent* agent) {
				float min_dist = INF;
			
				float2 forw = rotate2(agent->cit->_rot) * float2(0,1);
				float cone_dot = cos(cone);

				bool has_priority = false;
				bool priority = false;
			
				auto check = [&] (Agent* other) {
					assert(agent != other);
					
					float2 offs = other->cit->_pos - agent->cit->_pos;
					float2 dir = normalize(offs);
					
					float y = dot(forw, dir);
					
					//if (y > cone_dot) { // in cone
					if (y > 0.0f) { // in cone
						min_dist = min(min_dist, length(offs));
						priority = has_priority;
					}
				};
				
				// checks car against all higher priority cars (cars that came first)
				if (next_agents) {
					for (auto& a : next_agents->list) {
						assert(a != agent); // cant be in this list
						check(a);
					}
				}
				if (agents) {
					for (auto& a : agents->list) {
						//// don't check ourself or any cars that cam after us
						//if (a == agent) break;
						if (a == agent)
							break;
						else
							check(a);
					}
				}

				min_dist -= CAR_SIZE + 1;

				return brake_for_dist(min_dist);
			};
			
			assert(agent->cur_t < 1.0f);

			auto s = get_agent_state(agent, agent->idx);

			{ // move
				float brake = brake_for_cars(s.cur_agents, s.next_agents, agent);
				brake = min(brake, agent->_break);

				auto bez = bezier3(agent->cur_t, (float2)s.bezier.a, (float2)s.bezier.b, (float2)s.bezier.c);
				float bez_speed = length(bez.vel); // delta pos / bezier t

			
				agent->cit->_pos = float3(bez.pos, s.bezier.a.z);
				agent->cit->_rot = bez_speed > 0 ? atan2f(-bez.vel.x, bez.vel.y) : 0; // atan with roated axes since rot=0 should be +y in my convention

				// (delta pos / delta time)[speed] * time[dt] / (delta pos / bezier delta t)[bez_speed]
				// -> delta t
				agent->cur_t += top_speed * app.input.dt * brake / bez_speed;
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

		for (auto& cit : app.entities.citizens) {
			if (cit->building) continue;
			cit->path->_break = 1;
		}

		for (auto& node : nodes) {
			update_node(node.get());
		}

		for (auto& cit : app.entities.citizens) {
			if (cit->building) {
				do_pathfind(cit.get());
			}
			else {
				update_vehicle(cit->path.get());
			}
		}
	}

	debug_node(app.selection.get<Node*>());
	debug_citizen(app.selection.get<Citizen*>());

	static RunningAverage pathings_avg(30);
	pathings_avg.push((float)pathing_count);
	float min, max;
	float avg = pathings_avg.calc_avg(&min, &max);
	ImGui::Text("pathing_count: avg %3.1f min: %3.1f max: %3.1f", avg, min, max);
}

} // namespace network
