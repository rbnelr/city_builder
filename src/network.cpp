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
	float pos_z;

	network::AgentList<Agent*>* cur_agents = nullptr;
	network::AgentList<Agent*>* next_agents = nullptr;

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
		s.pos_z = s0.z;
	}
	else if (idx == num_moves-1) {
		auto e_seg = agent->segments.back();
		auto e_lane = e_seg.clac_lane_info();
		float3 e0 = (e_lane.a + e_lane.b) * 0.5f;
		float3 e1 = agent->end->pos;

		s.state = ENTER_BUILDING;

		s.bezier = { e0, (e0+e1)*0.5f, e1 };
		s.pos_z = e1.z;
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

			s.bezier = { l.a, (l.a+l.b)*0.5f, l.b }; // 3d 2d?
			s.pos_z = l.a.z;
		}
		else {
			assert(seg && node && seg2);

			if (agent->idx == idx)
				assert(contains(s.cur_node->agents.free.list, agent));

			s.state = NODE;

			s.cur_agents = &node->agents.free;
			s.next_agents = &seg2->seg->agents.lanes[seg2->lane];

			s.bezier = calc_curve(l, l2);
			s.pos_z = l.a.z;
		}
	}

	return s;
}
	
float brake_for_dist (float obstacle_dist) {
	return clamp(map(obstacle_dist, 0.0f, 8.0f), 0.0f, 1.0f);
}

void brake_for_leading_car (Agent* cur, Agent* leading) {
	float dist = distance(cur->cit->front_pos, leading->cit->rear_pos) - 2; // safety dist

	cur->brake = min(cur->brake, brake_for_dist(dist));
}

void debug_node (App& app, Node* node) {
	if (!node) return;

	g_dbgdraw.wire_circle(node->pos, node->radius, lrgba(1,1,0,1));

	//{
	//	int i=0;
	//	for (auto& agent : node->agents.free.list) {
	//		g_dbgdraw.wire_circle(agent->cit->center(), CAR_SIZE*0.5f, lrgba(1,0,0.5f,1));
	//
	//		g_dbgdraw.text.draw_text(prints("%d", i++),
	//			30, 1, g_dbgdraw.text.map_text(agent->cit->center(), app.view));
	//	}
	//}
	{
		int i=0;
		for (auto& agent : node->agents.test.list) {
			g_dbgdraw.wire_circle(agent.agent->cit->center(), CAR_SIZE*0.5f, lrgba(1,0,0.5f,1));
	
			g_dbgdraw.text.draw_text(prints("%d", i++),
				30, 1, g_dbgdraw.text.map_text(agent.agent->cit->center(), app.view));
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

void debug_citizen (App& app, Citizen* cit) {
	if (!cit || !cit->path) return;

	float start_t = cit->path->front_t;
	for (int i=cit->path->idx; ; ++i) {
		auto s = get_agent_state(cit->path.get(), i);

		s.bezier.dbg_draw(app.view, 0, 5, lrgba(1,1,0,1), start_t, s.end_t); 

		start_t = s.next_start_t;
		if (s.state == ENTER_BUILDING) break;
	}
}

void dbg_brake_for_point (App& app, Agent* cur, float2 point, lrgba col = lrgba(1,0.5f,0,1)) {
	if (app.selection.get<Citizen*>() != cur->cit) return;

	g_dbgdraw.point(float3(point,0), 0.2f, col);
}

bool check_conflict (Connection const& a, Connection const& b,
		float* out_u0, float* out_v0, float* out_u1, float* out_v1,
		bool dbg) {
	
	if (a == b) {
		*out_u0 = 0;
		*out_v0 = 0;
		*out_u1 = 1;
		*out_v1 = 1;
		return true;
	}

	auto gen_points = [&] (float2* points, Connection const& conn, float shift) {
		auto bez = calc_curve(conn.a.clac_lane_info(shift), conn.b.clac_lane_info(shift));

		points[0] = bez.a;
		for (int i=0; i<COLLISION_STEPS; ++i) {
			float t = (float)(i+1) / COLLISION_STEPS;
			points[i+1] = bez.eval(t).pos;
		}
	};

	float2 a_pointsL[COLLISION_STEPS+1];
	float2 a_pointsR[COLLISION_STEPS+1];
	float2 b_pointsL[COLLISION_STEPS+1];
	float2 b_pointsR[COLLISION_STEPS+1];
	gen_points(a_pointsL, a, -LANE_COLLISION_R);
	gen_points(a_pointsR, a, +LANE_COLLISION_R);
	gen_points(b_pointsL, b, -LANE_COLLISION_R);
	gen_points(b_pointsR, b, +LANE_COLLISION_R);

	float u0 = INF;
	float v0 = INF;
	float u1 = -INF;
	float v1 = -INF;

	for (int i=0; i<COLLISION_STEPS; ++i) {
		for (int j=0; j<COLLISION_STEPS; ++j) {
			
			auto intersect = [&] (float2* a, float2* b) {
				float line_u, line_v;
				if (line_line_seg_intersect(a[i], a[i+1], b[j], b[j+1], &line_u, &line_v)) {
					float u = (line_u + (float)i) / COLLISION_STEPS;
					float v = (line_v + (float)j) / COLLISION_STEPS;

					u0 = min(u0, u);
					v0 = min(v0, v);
					u1 = max(u1, u);
					v1 = max(v1, v);

					return true;
				}
				return false;
			};

			intersect(a_pointsL, b_pointsL);
			intersect(a_pointsR, b_pointsL);
			intersect(a_pointsL, b_pointsR);
			intersect(a_pointsR, b_pointsR);
		}
	}
	
	*out_u0 = u0;
	*out_v0 = v0;
	*out_u1 = u1;
	*out_v1 = v1;

	if (dbg) {
		for (int i=0; i<COLLISION_STEPS; ++i) {
			g_dbgdraw.line(float3(a_pointsL[i],0), float3(a_pointsL[i+1],0), lrgba(1,1,0,1));
			g_dbgdraw.line(float3(a_pointsR[i],0), float3(a_pointsR[i+1],0), lrgba(1,1,0,1));
			g_dbgdraw.line(float3(b_pointsL[i],0), float3(b_pointsL[i+1],0), lrgba(0,1,1,1));
			g_dbgdraw.line(float3(b_pointsR[i],0), float3(b_pointsR[i+1],0), lrgba(0,1,1,1));
		}

		auto draw_line = [&] (float2* L, float2* R, float t) {
			int i = (int)(t * COLLISION_STEPS);
			t = t * COLLISION_STEPS - i;

			g_dbgdraw.line(
				float3(lerp(L[i], L[i+1], t), 0),
				float3(lerp(R[i], R[i+1], t), 0), lrgba(1,0,0,1));
		};
		if (u0 < INF) {
			draw_line(a_pointsL, a_pointsR, u0);
			draw_line(a_pointsL, a_pointsR, u1);
			draw_line(b_pointsL, b_pointsR, v0);
			draw_line(b_pointsL, b_pointsR, v1);
		}
	}

	return u0 < INF;
}

void update_segment (App& app, Segment* seg) {
	for (auto& lane : seg->agents.lanes) {
		// brake for car in front
		for (int i=1; i<(int)lane.list.size(); ++i) {
			Agent* prev = lane.list[i-1];
			Agent* cur = lane.list[i];
			
			dbg_brake_for_point(app, cur, (float2)prev->cit->rear_pos);
			if (cur->front_t < prev->rear_t) {
				brake_for_leading_car(cur, prev);
			}
			else {
				cur->brake = brake_for_dist(0);
			}
		}
	}
};

void update_node (App& app, Node* node) {
	//struct PossibleConflict {
	//	bool has_conflict;
	//	float a_t, b_t;
	//};
	//Hashmap<Connection, float, PossibleConflict> conflicts;

	Hashmap<SegLane, float, SegLaneHasher> avail_space;


	auto dbg_avail_space = [&] (SegLane const& lane_out, Agent* a) {
		if (app.selection.get<Node*>() != node) return;

		auto li = lane_out.clac_lane_info();
				
		auto pos = lerp(li.a, li.b, avail_space[lane_out] / lane_out.seg->lane_length);
		g_dbgdraw.point(pos, 1, lrgba(a->cit->col,1));
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

	auto yield_for_car = [&] (NodeAgents::NodeAgent& a, NodeAgents::NodeAgent& b) {
		assert(a != b);
		
		auto* sel  = app.selection .get<Citizen*>() ? app.selection .get<Citizen*>()->path.get() : nullptr;
		auto* sel2 = app.selection2.get<Citizen*>() ? app.selection2.get<Citizen*>()->path.get() : nullptr;
		bool dbg = (a.agent == sel || a.agent == sel2) && (b.agent == sel || b.agent == sel2);
		
		if (dbg) {
			printf("");
		}
	
		float a_t0, b_t0, a_t1, b_t1;
		if (!check_conflict(a.conn, b.conn, &a_t0, &b_t0, &a_t1, &b_t1, dbg))
			return;
		
		auto as = get_agent_state(a.agent, a.agent->idx);
		auto bs = get_agent_state(b.agent, b.agent->idx);

		float a_t_offs = as.state == SEGMENT ? -1.0f : 0; // if on segment then on segment before since segment after cars will not be iterated
		float b_t_offs = bs.state == SEGMENT ? -1.0f : 0; // if on segment then on segment before since segment after cars will not be iterated

		bool a_entered = a_t_offs + a.agent->front_t >= a_t0;
		bool a_exited  = a_t_offs + a.agent->rear_t  >= a_t1;
		bool b_entered = b_t_offs + b.agent->front_t >= b_t0;
		bool b_exited  = b_t_offs + b.agent->rear_t  >= b_t1;
	
		if (a_exited || b_exited)
			return;
		//if (a->blocked || b->blocked)
		//	return;
	
		if (a.conn.a == b.conn.a || a.conn.b == b.conn.b) {
			brake_for_leading_car(a.agent, b.agent);
			dbg_brake_for_point(app, a.agent, b.agent->cit->rear_pos);
			return;
		}

		float2 point = calc_curve(a.conn.a.clac_lane_info(), a.conn.b.clac_lane_info()).eval(a_t0).pos;
		float dist = distance(a.agent->cit->front_pos, point);
	
		if (dbg && !a_entered) g_dbgdraw.line(a.agent->cit->front_pos, float3(point,0), lrgba(1,0,0,1));
		
		//auto s = get_agent_state(a, a->idx);
		//if (s.state == SEGMENT) {
		//	float2 lane_end = (float2)s.seg_before_node->clac_lane_info().b;
		//	
		//	dist = distance(a->cit->front_pos, lane_end) + 0.2f;
		//	a->blocked = true;
		//}
		a.agent->brake = min(a.agent->brake, brake_for_dist(dist));
	
		dbg_brake_for_point(app, a.agent, point);
	};

	int count = (int)node->agents.test.list.size();
	for (int j=1; j<count; ++j) {
		auto a = node->agents.test.list[j];
		for (int i=0; i<j; ++i) {
			auto b = node->agents.test.list[i];

			yield_for_car(a, b);
		}
	}

	for (auto a : node->agents.test.list) {
		auto& target_lane = a.conn.b.seg->agents.lanes[a.conn.b.lane].list;
		if (!target_lane.empty()) {
			auto* b = target_lane.back();

			brake_for_leading_car(a.agent, b);
			dbg_brake_for_point(app, a.agent, b->cit->rear_pos);
		}
	}
	
	// avail space in out lane logic
	
	//for (int i=0; i<(int)node->agents.free.list.size(); ++i) {
	//	auto* a = node->agents.free.list[i];
	//			
	//	// count space in target lane
	//	auto s = get_agent_state(a, a->idx);
	//	dbg_avail_space(*s.seg_after_node, a);
	//	avail_space[*s.seg_after_node] -= CAR_SIZE + 1; // Use real car length for specific car here
	//}
	//
	//for (auto& lane : node->in_lanes) {
	//	for (auto* agent : lane.seg->agents.lanes[lane.lane].list) {
	//		auto s = get_agent_state(agent, agent->idx);
	//		if (!s.seg_after_node) continue;
	//
	//		if (avail_space[*s.seg_after_node] < CAR_SIZE) {
	//			float2 lane_end = (float2)lane.clac_lane_info().b;
	//				
	//			float dist = distance(agent->cit->front_pos, lane_end) + 0.2f;
	//			agent->brake = min(agent->brake, brake_for_dist(dist));
	//			agent->blocked = true;
	//
	//			dbg_brake_for_point(app, agent, lane_end, lrgba(0.2f,0.8f,1,1));
	//		}
	//		else {
	//			dbg_avail_space(*s.seg_after_node, agent);
	//			avail_space[*s.seg_after_node] -= CAR_SIZE + 1;
	//		}
	//	}
	//}
	
	////
	//for (int i=0; i<(int)node->agents.free.list.size(); ++i) {
	//	auto* a = node->agents.free.list[i];
	//	auto conn_a = get_conn(a);
	//	
	//	// process remaining node agents
	//	for (int j=i+1; j<(int)node->agents.free.list.size(); ++j) {
	//		auto* b = node->agents.free.list[j];
	//		vehicle_conflict(b, a, get_conn(b), conn_a);
	//	}
	//			
	//	// process last agents in outgoing lanes
	//	for (auto& out_lane : node->out_lanes) {
	//		auto& agents = out_lane.seg->agents.lanes[out_lane.lane].list;
	//		if (!agents.empty()) {
	//			vehicle_conflict(a, agents.back(), conn_a, get_conn2(agents.back()));
	//		}
	//	}
	//
	//	// process ingoing lanes
	//	for (auto& in_lane : node->in_lanes) {
	//		for (auto& b : in_lane.seg->agents.lanes[in_lane.lane].list) {
	//			vehicle_conflict(b, a, get_conn(b), conn_a);
	//		}
	//	}
	//}
	//
	//// process ingoing lanes
	//for (int i=0; i<(int)node->in_lanes.size()-1; ++i) {
	//	auto& lane = node->in_lanes[i];
	//	for (auto& a : lane.seg->agents.lanes[lane.lane].list) {
	//		auto conn_a = get_conn(a);
	//
	//		// process remaining ingoing lanes
	//		for (int j=i+1; j<(int)node->in_lanes.size(); ++j) {
	//			auto& other_lane = node->in_lanes[j];
	//			for (auto& b : other_lane.seg->agents.lanes[other_lane.lane].list) {
	//				vehicle_conflict(a, b, conn_a, get_conn(b));
	//			}
	//		}
	//	}
	//}
};
		
void update_vehicle (App& app, Agent* agent) {
	if (app.sim_paused)
		return;

	assert(agent->front_t < 1.0f);
	
	auto s = get_agent_state(agent, agent->idx);

	// (delta pos / delta time)[speed] * time[dt] / (delta pos / bezier delta t)[bez_speed]
	// -> delta t
	float dp = app.net.top_speed * app.input.dt * agent->brake;
	agent->front_t += dp * agent->bez_speed;

	if (agent->front_t >= s.end_t) {
		agent->idx++;
		agent->front_t = s.next_start_t;

		if (s.cur_agents)  s.cur_agents ->remove(agent);
		if (s.next_agents) s.next_agents->add(agent);

		if (s.state == ENTER_BUILDING) {
			// end path
			agent->cit->building = agent->end;
			agent->cit->path = nullptr;
			return;
		}

		if (s.state == NODE) {
			assert(s.cur_node);
			s.cur_node->agents.test.remove(NodeAgents::NodeAgent{ agent, {} });
		}

		s = get_agent_state(agent, agent->idx);

		if (s.state == SEGMENT && s.cur_node) {
			assert(s.seg_before_node && s.seg_after_node);
			s.cur_node->agents.test.add(NodeAgents::NodeAgent{ agent, { *s.seg_before_node, *s.seg_after_node } });
		}
	}

	// at front
	auto bez_res = s.bezier.eval(agent->front_t);
	agent->bez_speed = 1.0f / length(bez_res.vel); // bezier t / delta pos

	agent->rear_t = agent->front_t - CAR_SIZE * agent->bez_speed;

	float2 new_front = bez_res.pos;
	
	float2 old_front = (float2)agent->cit->front_pos;
	float2 old_rear  = (float2)agent->cit->rear_pos;

	float2 forw = normalizesafe(old_front - old_rear);
	//float forw_amount = dot(new_front - old_front, forw);

	float2 ref_point = old_rear + CAR_SIZE*app.net.rear_test * forw; // Kinda works to avoid goofy car rear movement?

	float2 new_rear = new_front - normalizesafe(new_front - ref_point) * CAR_SIZE;

	agent->cit->front_pos = float3(new_front, s.pos_z);
	agent->cit->rear_pos = float3(new_rear, s.pos_z);
};
		

void Network::simulate (App& app) {
	ZoneScoped;

	pathing_count = 0;

	auto do_pathfind = [&] (Citizen* cit) {
		auto* cur_target = app.entities.buildings[ app.test_rand.uniformi(0, (int)app.entities.buildings.size()) ].get();
	
		cit->front_pos = cit->building->pos;
		cit->rear_pos = cit->front_pos;

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
		{
			ZoneScopedN("init pass");
			for (auto& cit : app.entities.citizens) {
				if (cit->building) continue;
				cit->path->brake = 1;
				cit->path->blocked = false;
			}
		}
		
		{
			ZoneScopedN("update segments");
			for (auto& seg : segments) {
				update_segment(app, seg.get());
			}
		}
		{
			ZoneScopedN("update nodes");
			for (auto& node : nodes) {
				update_node(app, node.get());
			}
		}
		
		{
			ZoneScopedN("final pass");
			for (auto& cit : app.entities.citizens) {
				if (cit->building) {
					do_pathfind(cit.get());
				}
				else {
					update_vehicle(app, cit->path.get());
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
