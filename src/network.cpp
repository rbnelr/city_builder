#include "common.hpp"
#include "network.hpp"
#include "app.hpp"

namespace network {

bool Network::pathfind (Segment* start, Segment* target, Agent* agent) {
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
						other_node->_pred_seg  = { seg, (uint16_t)i };
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
	agent->nodes.push_back(start_node);

	// start segment
	// if start node is segment.a then we go from b->a and thus need to start on a reverse lane
	int start_lane = start_node == start->node_b ? 0 : (int)start->layout->lanes.size()-1;
	agent->segments.push_back({ start, (uint16_t)start_lane });

	// nodes
	for (int i=(int)tmp_nodes.size()-2; i>=0; --i) {
		assert(tmp_nodes[i]->_pred_seg.seg);
		// segment of predecessor to path node
		agent->segments.push_back( tmp_nodes[i]->_pred_seg );
		// path node
		agent->nodes.push_back(tmp_nodes[i]);
	}

	// end segment
	// if end node is segment.a then we go from a->b and thus need to start on a forward lane
	int end_lane = end_node == target->node_a ? 0 : (int)target->layout->lanes.size()-1;
	agent->segments.push_back({ target, (uint16_t)end_lane });

	return true;
}

inline Bezier3 calc_curve (Line const& l0, Line const& l1) {
	float2 point;
	if (!line_line_intersect((float2)l0.a, (float2)l0.b, (float2)l1.a, (float2)l1.b, &point))
		point = (l0.b+l1.a)*0.5f;
		//point = (l0.b+l1.a)*0.5f + 0.5f * distance(l0.a, l1.b) * normalizesafe(l0.b - l0.a);
					
	return { l0.b, float3(point, l0.a.z), l1.a };
}

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

		s.state = AgentState::EXIT_BUILDING;
		s.next_start_t = 0.5f;

		s.next_agents = &s_seg.agents().list;

		s.bezier = { s0, (s0+s1)*0.5f, s1 };
		s.pos_z = s0.z;
	}
	else if (idx == num_moves-1) {
		auto e_seg = agent->segments.back();
		auto e_lane = e_seg.clac_lane_info();
		float3 e0 = (e_lane.a + e_lane.b) * 0.5f;
		float3 e1 = agent->end->pos;

		s.state = AgentState::ENTER_BUILDING;

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
				assert(contains(s.seg_before_node->agents().list.list, agent));

			s.state = AgentState::SEGMENT;

			s.cur_agents = &seg->agents().list;
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

			s.state = AgentState::NODE;

			s.cur_agents = &node->agents.free;
			s.next_agents = &seg2->agents().list;

			s.bezier = calc_curve(l, l2);
			s.pos_z = l.a.z;
		}
	}

	return s;
}

//AgentState _FORCEINLINE get_agent_state_only_conn (Agent* agent, int idx) {
//	AgentState s;
//
//	int num_nodes  = (int)agent->nodes.size();
//	int num_seg    = (int)agent->segments.size();
//	int num_moves = num_nodes + num_seg + 2;
//
//	assert(num_nodes >= 1);
//	assert(num_nodes + 1 == num_seg);
//	assert(idx < num_moves);
//
//	if (idx == 0) {
//		
//	}
//	else if (idx == num_moves-1) {
//		
//	}
//	else {
//		int i = idx - 1;
//				
//		bool last_seg = i/2+1 >= num_seg;
//		assert(i/2 < num_nodes == !last_seg);
//
//		auto* seg = &agent->segments[i/2];
//		auto* node = !last_seg ? agent->nodes[i/2] : nullptr;
//		auto* seg2 = !last_seg ? &agent->segments[i/2+1] : nullptr;
//
//		s.cur_node = node;
//		s.seg_before_node = seg;
//		s.seg_after_node = seg2;
//
//		if (node) assert(contains(node->in_lanes, *s.seg_before_node));
//		if (node) assert(contains(node->out_lanes, *s.seg_after_node));
//	}
//
//	return s;
//}
	
float _brake_for_dist (float obstacle_dist) {
	return clamp(map(obstacle_dist, 0.0f, 8.0f), 0.0f, 1.0f);
}
void brake_for_dist (Agent* agent, float obstacle_dist) {
	float brake = _brake_for_dist(obstacle_dist);
	agent->brake = min(agent->brake, brake);
}

//void brake_for_leading_car (Agent* cur, Agent* leading) {
//	float dist = distance(cur->cit->front_pos, leading->cit->rear_pos) - 2; // safety dist
//
//	cur->brake = min(cur->brake, brake_for_dist(dist));
//}

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

	ImGui::Text("%d conflicts cached", (int)node->agents.conflict_cache.size());

	//{
	//	Hashmap<SegLane, lrgba, SegLaneHasher> cols;
	//	int col_i = 0;
	//	auto new_col = [&] () -> lrgba {
	//		return g_dbgdraw.COLS[col_i++ % ARRLEN(g_dbgdraw.COLS)];
	//	};
	//
	//	for (auto& kv : node->agents.conflict_cache) {
	//		auto col0 = cols.get_or_create(kv.first.a.a, new_col);
	//		auto col1 = cols.get_or_create(kv.first.a.b, new_col);
	//		auto col2 = cols.get_or_create(kv.first.b.a, new_col);
	//		auto col3 = cols.get_or_create(kv.first.b.b, new_col);
	//	
	//		ImGui::TextColored(col0, "%p:%d",	kv.first.a.a.seg, kv.first.a.a.lane); ImGui::SameLine();
	//		ImGui::TextColored(col1, "-%p:%d",	kv.first.a.b.seg, kv.first.a.b.lane); ImGui::SameLine();
	//		ImGui::Text(" | "); ImGui::SameLine();
	//		ImGui::TextColored(col2, "%p:%d",	kv.first.b.a.seg, kv.first.b.a.lane); ImGui::SameLine();
	//		ImGui::TextColored(col3, "-%p:%d",	kv.first.b.b.seg, kv.first.b.b.lane);
	//	}
	//	
	//	for (auto& kv : cols) {
	//		auto p = kv.first.clac_lane_info();
	//		g_dbgdraw.line(p.a, p.b, kv.second);
	//	}
	//}
}
void debug_citizen (App& app, Citizen* cit) {
	if (!cit || !cit->agent) return;

	float start_t = cit->agent->bez_t;
	for (int i=cit->agent->idx; ; ++i) {
		auto s = get_agent_state(cit->agent.get(), i);

		s.bezier.dbg_draw(app.view, 0, 5, lrgba(1,1,0,1), start_t, s.end_t); 

		start_t = s.next_start_t;
		if (s.state == AgentState::ENTER_BUILDING) break;
	}

	ImGui::Separator();
	ImGui::TextColored(lrgba(cit->col, 1), "debug citizen");

	static circular_buffer<float> speed_buf = circular_buffer<float>(500);
	speed_buf.push(cit->agent->speed);

	ImGui::PlotLines("speed", speed_buf.data(), (int)speed_buf.count(), 0, 0, 0.0f, app.net.top_speed, float2(0, 200));
}

void dbg_brake_for (App& app, Agent* cur, float dist, float3 obstacle, lrgba col) {
	// dir does not actually point where we are going to stop
	// obsticle visualizes what object we are stopping for
	// dist is approx distance along bezier to stop at, which we don't bother visualizing exactly

	dist = max(dist, 0.0f);

	float3 pos = cur->cit->front_pos;
	float3 dir = normalizesafe(obstacle - pos);
	float3 end = pos + float3(dir,0)*dist;
	float3 normal = float3(rotate90(dir), 0);

	g_dbgdraw.arrow(app.view, pos, obstacle - pos, 0.3f, col);
	g_dbgdraw.line(end - normal, end + normal, col);
}
void _FORCEINLINE dbg_brake_for_agent (App& app, Agent* cur, float dist, Agent* obstacle) {
	if (app.selection.get<Citizen*>() == cur->cit) {
		float3 center = (obstacle->cit->rear_pos + obstacle->cit->front_pos) * 0.5f;
		dbg_brake_for(app, cur, dist, center, lrgba(1,0.1f,0,1));
	}
}
void _FORCEINLINE dbg_brake_for_blocked_lane (App& app, NodeAgent& a, float dist) {
	if (app.selection.get<Citizen*>() == a.agent->cit) {
		dbg_brake_for(app, a.agent, dist, a.conn.conn.b.clac_lane_info().a, lrgba(0.2f,0.8f,1,1));
	}
}

auto _gen_points (float2* points, Connection const& conn, float shift) {
	auto bez = calc_curve(conn.a.clac_lane_info(shift), conn.b.clac_lane_info(shift));
	//auto co = bez.get_coeff();

	points[0] = bez.a;
	for (int i=0; i<COLLISION_STEPS; ++i) {
		float t = (float)(i+1) / COLLISION_STEPS;
		points[i+1] = bez.eval_value_fast_t(t);
	}
}
Conflict check_conflict (CachedConnection const& a, CachedConnection const& b) {
	assert(a.conn != b.conn);
	
	float u0 = INF;
	float v0 = INF;
	float u1 = -INF;
	float v1 = -INF;

	for (int i=0; i<COLLISION_STEPS; ++i) {
		float2 aL_dir = a.pointsL[i+1] - a.pointsL[i];
		float2 aR_dir = a.pointsR[i+1] - a.pointsR[i];

		for (int j=0; j<COLLISION_STEPS; ++j) {
			float2 bL_dir = b.pointsL[j+1] - b.pointsL[j];
			float2 bR_dir = b.pointsR[j+1] - b.pointsR[j];
			
			float line_u, line_v;

			if (line_line_seg_intersect(a.pointsL[i], aL_dir, b.pointsL[j], bL_dir, &line_u, &line_v)) {
				float u = line_u + (float)i;
				float v = line_v + (float)j;

				u0 = min(u0, u);
				v0 = min(v0, v);
				u1 = max(u1, u);
				v1 = max(v1, v);
			}

			if (line_line_seg_intersect(a.pointsR[i], aR_dir, b.pointsL[j], bL_dir, &line_u, &line_v)) {
				float u = line_u + (float)i;
				float v = line_v + (float)j;

				u0 = min(u0, u);
				v0 = min(v0, v);
				u1 = max(u1, u);
				v1 = max(v1, v);
			}

			if (line_line_seg_intersect(a.pointsL[i], aL_dir, b.pointsR[j], bR_dir, &line_u, &line_v)) {
				float u = line_u + (float)i;
				float v = line_v + (float)j;

				u0 = min(u0, u);
				v0 = min(v0, v);
				u1 = max(u1, u);
				v1 = max(v1, v);
			}

			if (line_line_seg_intersect(a.pointsR[i], aR_dir, b.pointsR[j], bR_dir, &line_u, &line_v)) {
				float u = line_u + (float)i;
				float v = line_v + (float)j;

				u0 = min(u0, u);
				v0 = min(v0, v);
				u1 = max(u1, u);
				v1 = max(v1, v);
			}
		}
	}

	u0 *= 1.0f / COLLISION_STEPS;
	u1 *= 1.0f / COLLISION_STEPS;
	v0 *= 1.0f / COLLISION_STEPS;
	v1 *= 1.0f / COLLISION_STEPS;

	return { u0, u1, v0, v1 };
}

void debug_conflict (CachedConnection const& a, CachedConnection const& b, Conflict& conf) {
	for (int i=0; i<COLLISION_STEPS; ++i) {
		g_dbgdraw.line(float3(a.pointsL[i],0), float3(a.pointsL[i+1],0), lrgba(1,1,0,1));
		g_dbgdraw.line(float3(a.pointsR[i],0), float3(a.pointsR[i+1],0), lrgba(1,1,0,1));
		g_dbgdraw.line(float3(b.pointsL[i],0), float3(b.pointsL[i+1],0), lrgba(0,1,1,1));
		g_dbgdraw.line(float3(b.pointsR[i],0), float3(b.pointsR[i+1],0), lrgba(0,1,1,1));
	}

	auto draw_line = [&] (float2 const* L, float2 const* R, float t) {
		int i = (int)(t * COLLISION_STEPS);
		t = t * COLLISION_STEPS - i;

		g_dbgdraw.line(
			float3(lerp(L[i], L[i+1], t), 0),
			float3(lerp(R[i], R[i+1], t), 0), lrgba(1,0,0,1));
	};
	if (conf) {
		draw_line(a.pointsL, a.pointsR, conf.a_t0);
		draw_line(a.pointsL, a.pointsR, conf.a_t1);
		draw_line(b.pointsL, b.pointsR, conf.b_t0);
		draw_line(b.pointsL, b.pointsR, conf.b_t1);
	}
}

Conflict query_conflict (Node* node, CachedConnection const& a, CachedConnection const& b) {
	if (a.conn == b.conn)
		return { 0,1, 0,1 }; // never cache overlapping paths
	
	// order a/b in some order in hashmap key to save 50% of (symmetrical) conflicts
	bool order = a.conn < b.conn;

	auto* pa = &a;
	auto* pb = &b;
	if (!order) std::swap(pa, pb);
	
	ConflictKey key = ConflictKey{ pa->conn, pb->conn };

	auto conf = node->agents.conflict_cache.get_or_create(key, [&] () -> Conflict {
		return check_conflict(*pa, *pb); // cache ordered conflict
	});

	// reverse ordering for result, effectively reusing b->a conflict as a->b
	return order ? conf : Conflict{ conf.b_t0, conf.b_t1, conf.a_t0, conf.a_t1 };
}

void update_segment (App& app, Segment* seg) {
	for (auto& lane : seg->agents.lanes) {
		// brake for car in front
		for (int i=1; i<(int)lane.list.list.size(); ++i) {
			Agent* prev = lane.list.list[i-1];
			Agent* cur = lane.list.list[i];
			
			// approx seperation using cur car bez_speed
			float dist = (prev->bez_t - cur->bez_t) * cur->bez_speed - (CAR_SIZE + 1);

			brake_for_dist(cur, dist);
			dbg_brake_for_agent(app, cur, dist, prev);
		}
	}
}

void _yield_for_car (App& app, Node* node, NodeAgent& a, NodeAgent& b) {
	assert(a.agent != b.agent);
		
	auto* sel  = app.selection .get<Citizen*>() ? app.selection .get<Citizen*>()->agent.get() : nullptr;
	auto* sel2 = app.selection2.get<Citizen*>() ? app.selection2.get<Citizen*>()->agent.get() : nullptr;
		
	bool dbg = (a.agent == sel || a.agent == sel2) && (b.agent == sel || b.agent == sel2);
		
	//if (dbg) {
	//	printf("");
	//}


	auto conf = query_conflict(node, a.conn, b.conn);

	if (dbg) debug_conflict(a.conn, b.conn, conf);

	if (!conf)
		return;

	float a_k0 = conf.a_t0 * a.conn.bez_len;
	float a_k1 = conf.a_t1 * a.conn.bez_len;
	float b_k0 = conf.b_t0 * b.conn.bez_len;
	float b_k1 = conf.b_t1 * b.conn.bez_len;
		
	bool a_entered = a.front_k >= a_k0;
	bool a_exited  = a.rear_k  >= a_k1;
	bool b_entered = b.front_k >= b_k0;
	bool b_exited  = b.rear_k  >= b_k1;

	bool b_rear_entered = b.rear_k >= b_k0;
		
	bool diverge = a.conn.conn.a == b.conn.conn.a; // same start point
	bool merge   = a.conn.conn.b == b.conn.conn.b; // same end point
	//bool crossing = !merge && !diverge; // normal crossing
	bool same = merge && diverge; // identical path
		
	// check if conflict relevant
	// NOTE: special case of merge, where car exited should still be followed,
	// but this is handled by seperate check against outgoing lane, since  b.rear_k >= b_k1 puts it in the outgoing lane
	if (a_exited || b_exited)
		return;

	float stop_k = a_k0;
		
	// if same conn: follow car
	// if diverge:   follow car
	// if merge:     stop before conflict until b rear in conflict, then follow
	// if crossing:  stop before conflict
	if (same || diverge || (merge && b_rear_entered)) {
		// TODO: might want to do this differently if b_exited and we still follow
			
		// need to follow car, approx correct stop_k by mapping b rear from b's collision zone to a's zone
		stop_k = lerp(a_k0, a_k1, map(b.rear_k, b_k0, b_k1));
	}
		
	stop_k -= SAFETY_DIST;
	float dist = stop_k - a.front_k; // wait before conflict by default

	brake_for_dist(a.agent, dist);
	dbg_brake_for_agent(app, a.agent, dist, b.agent);
}

void update_node (App& app, Node* node) {
	bool node_dbg = app.selection.get<Node*>() == node;
	auto dbg_avail_space = [&] (SegLane const& lane_out, Agent* a) {
		auto li = lane_out.clac_lane_info();
		
		auto pos = lerp(li.a, li.b, lane_out.agents().avail_space / lane_out.seg->lane_length);
		g_dbgdraw.point(pos, 1, lrgba(a->cit->col,1));
	};

	//
	for (auto& lane_out : node->out_lanes) {
		auto& avail_space = lane_out.agents().avail_space;
		avail_space = lane_out.seg->lane_length;
		
		for (auto* a : lane_out.seg->agents.lanes[lane_out.lane].list.list) {
			if (node_dbg) dbg_avail_space(lane_out, a);

			avail_space -= CAR_SIZE + SAFETY_DIST;
		}
	}
	
	// Track cars that are relevant to intersection
	for (auto& lane : node->in_lanes) {
		for (auto* agent : lane.agents().list.list) {
			if (node->agents.test.contains(agent)) continue; // TODO: Expensive contains with vector

			float dist = (1.0f - agent->bez_t) * agent->bez_speed;
			if (dist > 10.0f) break;

			//auto s = get_agent_state_only_conn(agent, agent->idx);
			if (!agent->state.seg_before_node || !agent->state.seg_after_node)
				continue;

			NodeAgent a;
			a.agent = agent;
			a.node_idx = agent->idx+1;

			a.conn.conn = { *agent->state.seg_before_node, *agent->state.seg_after_node };
			auto bez = calc_curve(a.conn.conn.a.clac_lane_info(), a.conn.conn.b.clac_lane_info());
			a.conn.bez_len = bez.approx_len(COLLISION_STEPS);
			
			_gen_points(a.conn.pointsL, a.conn.conn, -LANE_COLLISION_R);
			_gen_points(a.conn.pointsR, a.conn.conn, +LANE_COLLISION_R);

			node->agents.test.add(a);
		}
	}
	node->agents.test.remove_if([&] (NodeAgent& a) {
		if (a.agent->idx > a.node_idx+1)
			return true; // past outgoing lane
		if (a.agent->idx == a.node_idx+1) {
			// in outgoing lane
			float dist = a.agent->bez_t * a.agent->bez_speed;
			if (dist > CAR_SIZE)
				return true;
		}
		return false;
	});
	
	auto update_ks = [&] (NodeAgent& a) {

		// ingoing lane
		if (a.agent->idx == a.node_idx-1) {
			// extrapolate and map from negative to 0
			a.front_k = (a.agent->bez_t - 1.0f) * a.agent->bez_speed;
		}
		// on node
		else if (a.agent->idx == a.node_idx) {
			// approximate by just mapping t (which is wrong)
			a.front_k = a.agent->bez_t * a.conn.bez_len;
		}
		// outgoing lane
		else {
			assert(a.agent->idx == a.node_idx+1);
			// extrapolate and map from negative to 0
			a.front_k = a.agent->bez_t * a.agent->bez_speed + a.conn.bez_len;
		}
		
		a.rear_k = a.front_k - CAR_SIZE;
	};
	
	
	// allocate space in priority order and remember blocked cars
	for (auto& a : node->agents.test.list) {
		update_ks(a);

		if (a.agent->idx > a.node_idx) {
			// already in outgoing lane (don't need to wait and avoid counting avail space twice)
			continue;
		}

		auto& avail_space = a.conn.conn.b.agents().avail_space;
		if (avail_space < CAR_SIZE) {
			float dist = -a.front_k; // end of ingoing lane
			
			brake_for_dist(a.agent, dist);
			dbg_brake_for_blocked_lane(app, a, dist);

			a.agent->blocked = true; // WARNING: This is not threadsafe if we want to thread nodes/segments individually
		}
		else {
			if (node_dbg) dbg_avail_space(a.conn.conn.b, a.agent);
			avail_space -= CAR_SIZE + SAFETY_DIST;

			a.agent->blocked = false; // explicitly unblock
		}
	}

	//// swap cars
	//for (int i=0; i<(int)node->agents.test.list.size()-1; ++i) {
	//	auto& a = node->agents.test.list[i];
	//	auto& b = node->agents.test.list[i+1];
	//
	//	bool diverge = a.conn.a == b.conn.a; // same start point
	//	if (diverge) continue;
	//
	//	auto ya = test_agent(a);
	//	auto yb = test_agent(b);
	//
	//	// estimated time to leave intersection based on cur speed
	//	float eta_a = (ya.conn_len - ya.front_k) / (ya.agent->speed + 0.01f);
	//	float eta_b = (yb.conn_len - yb.front_k) / (yb.agent->speed + 0.01f);
	//	
	//	bool swap = eta_b < eta_a * 0.5f;
	//
	//	if (swap) {
	//		// make a yield to b
	//		// TODO: should check if a is not already blocking b's path
	//
	//		std::swap(a, b);
	//	}
	//}

	int count = (int)node->agents.test.list.size();
	for (int j=0; j<count; ++j) {
		auto& a = node->agents.test.list[j];
		for (int i=0; i<j; ++i) {
			auto& b = node->agents.test.list[i];

			_yield_for_car(app, node, a, b);
		}

		// brake for target lane car
		auto& target_lane = a.conn.conn.b.agents().list.list;
		if (!target_lane.empty() && target_lane.back() != a.agent) {
			float a_front_k = a.front_k - a.conn.bez_len; // relative to after node

			auto* b = target_lane.back();
			float b_rear_k = b->bez_t * b->bez_speed - CAR_SIZE;

			float dist = b_rear_k - a_front_k;
			dist -= SAFETY_DIST;
		
			brake_for_dist(a.agent, dist);
			dbg_brake_for_agent(app, a.agent, dist, b);
		}
	}
}

void update_vehicle (App& app, Agent* agent) {
	if (app.sim_paused)
		return;

	float dt = app.input.dt * app.sim_speed;

	assert(agent->bez_t < 1.0f);
	
	// car speed change
	float target_speed = app.net.top_speed * agent->brake;
	if (target_speed >= agent->speed) {
		agent->speed += app.net.car_accel * dt;
		agent->speed = min(agent->speed, target_speed);
	}
	else {
		agent->speed = target_speed; // brake instantly for now
	}
	
	// move car with speed on bezier based on previous frame delta t
	agent->bez_t += agent->speed * dt / agent->bez_speed;

	// do bookkeeping when car reaches end of current bezier
	if (agent->bez_t >= agent->state.end_t) {
		agent->idx++;
		agent->bez_t = agent->state.next_start_t;

		if (agent->state.cur_agents)  agent->state.cur_agents ->remove(agent);
		if (agent->state.next_agents) agent->state.next_agents->add(agent);

		if (agent->state.state == AgentState::ENTER_BUILDING) {
			// end path
			agent->cit->building = agent->end;
			agent->cit->agent = nullptr;
			return;
		}

		agent->state = get_agent_state(agent, agent->idx);
	}

	// eval bezier at car front
	auto bez_res = agent->state.bezier.eval(agent->bez_t);
	// remember bezier delta t for next frame
	agent->bez_speed = length(bez_res.vel); // bezier t / delta pos

	// actually move car rear using (bogus) trailer formula
	float2 new_front = bez_res.pos;
	
	float2 old_front = (float2)agent->cit->front_pos;
	float2 old_rear  = (float2)agent->cit->rear_pos;

	float2 forw = normalizesafe(old_front - old_rear);
	//float forw_amount = dot(new_front - old_front, forw);

	float2 ref_point = old_rear + CAR_SIZE*app.net.rear_test * forw; // Kinda works to avoid goofy car rear movement?

	float2 new_rear = new_front - normalizesafe(new_front - ref_point) * CAR_SIZE;

	agent->cit->front_pos = float3(new_front, agent->state.pos_z);
	agent->cit->rear_pos  = float3(new_rear,  agent->state.pos_z);
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

			auto agent = std::make_unique<network::Agent>();
			agent->cit = cit;
			agent->start = cit->building;
			agent->end   = cur_target;
			bool valid = pathfind(cit->building->connected_segment, cur_target->connected_segment, agent.get());
			if (valid) {
				cit->building = nullptr;
				cit->agent = std::move(agent);
				// get initial state
				cit->agent->state = get_agent_state(cit->agent.get(), cit->agent->idx);
				cit->agent->bez_speed = INF; // force not movement on initial tick
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
				cit->agent->brake = 1;
				cit->agent->blocked = false;
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
					update_vehicle(app, cit->agent.get());
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
