#include "common.hpp"
#include "network.hpp"
#include "app.hpp"

namespace network {

bool Network::pathfind (Segment* start, Segment* target, Agent* agent) {
	ZoneScoped;
	// use dijkstra

#define DIJK_OPT 0
#if DIJK_OPT
	std::vector<Node*> heap_vec;
	heap_vec.reserve(512);

	auto get_key = [] (Node* node) -> float { return node->_cost; };
	auto get_idx = [] (Node* node) -> int& {  return node->_q_idx; };
	MinHeapFunc<Node*, decltype(get_key), decltype(get_idx)> min_heap = {
		heap_vec, get_key, get_idx
	};
#else
	struct Queued {
		Node* node;
		float cost;
	};

	struct Comparer {
		bool operator () (Queued const& l, Queued const& r) {
			// true: l < r
			// but since priority_queue only lets us get
			// max element rather than min flip everything around
			return l.cost > r.cost;
		}
	};
	std::priority_queue<Queued, std::vector<Queued>, Comparer> unvisited;
#endif

	// queue all nodes
	for (auto& node : nodes) {
		node->_cost = INF;
		node->_visited = false;
		node->_q_idx = -1;
		node->_pred = nullptr;
		node->_pred_seg = nullptr;
	}

	// TODO: could actually start only on the lane facing the target building
	//  (ie. don't allow crossing the road middle)
	{ // handle the two start nodes
		// pretend start point is at center of start segment for now
		start->node_a->_cost = start->_length / 0.5f / start->asset->speed_limit;
		start->node_b->_cost = start->_length / 0.5f / start->asset->speed_limit;

		start->node_a->_pred_seg = start;
		start->node_b->_pred_seg = start;
	}

#if DIJK_OPT
	min_heap.push(start->node_a);
	min_heap.push(start->node_b);
#else
	unvisited.push({ start->node_a, start->node_a->_cost });
	unvisited.push({ start->node_b, start->node_b->_cost });
#endif

	_dijk_iter = 0;
	_dijk_iter_dupl = 0;
	_dijk_iter_lanes = 0;
	
#if DIJK_OPT
	while (!min_heap.items.empty()) {
#else
	while (!unvisited.empty()) {
#endif
		_dijk_iter_dupl++;
		
	#if DIJK_OPT
		// visit node with min cost
		Node* cur_node = min_heap.pop();

		assert(!cur_node->_visited);
	#else
		// visit node with min cost
		auto _cur_node = unvisited.top();
		Node* cur_node = _cur_node.node;
		unvisited.pop();

		if (cur_node->_visited) continue;
	#endif
		cur_node->_visited = true;

		_dijk_iter++;

		// early out optimization
		if (target->node_a->_visited && target->node_b->_visited)
			break; // shortest path found if both target segment nodes are visited

		// Get all allowed turns for incoming segment
		AllowedTurns allowed = (AllowedTurns)0;
		for (auto& lane : cur_node->_pred_seg->lanes) {
			allowed |= lane.allowed_turns;
		}

		// update neighbours with new minimum cost
		for (auto& lane : cur_node->out_lanes) {
			Node* other_node = lane.seg->get_other_node(cur_node);

			auto turn = classify_turn(cur_node, cur_node->_pred_seg, lane.seg);
			if ((allowed & turn) == 0) {
				// turn not allowed
				assert(false); // currently impossible, only the case for roads with no right turn etc.
				//continue;
			}

			float len = lane.seg->_length + lane.seg->node_a->_radius + lane.seg->node_b->_radius;
			float cost = len / lane.seg->asset->speed_limit;
			assert(cost > 0);

			float new_cost = cur_node->_cost + cost;
			if (new_cost < other_node->_cost && !other_node->_visited) {
				other_node->_pred      = cur_node;
				other_node->_pred_seg  = lane.seg;
				other_node->_cost      = new_cost;
				//assert(!other_node->_visited); // dijstra with positive costs should prevent this

			#if DIJK_OPT
				min_heap.push_or_decrease(other_node);
			#else
				unvisited.push({ other_node, other_node->_cost }); // push updated neighbour (duplicate)
			#endif
			}

			_dijk_iter_lanes++;
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

	std::vector<Segment*> reverse_segments;
	reverse_segments.push_back(target);

	Node* cur = end_node;
	while (cur) {
		assert(cur->_pred_seg);
		reverse_segments.push_back(cur->_pred_seg);
		cur = cur->_pred;
	}
	assert(reverse_segments.size() >= 1);

	for (int i=(int)reverse_segments.size()-1; i>=0; --i) {
		agent->path.push_back(reverse_segments[i]);
	}

	return true;
}

AgentState get_agent_state (Agent* agent, int idx) {
	AgentState s;

	int num_seg = (int)agent->path.size();
	int num_moves = num_seg + (num_seg-1) + 2;
	assert(num_seg >= 1);

	// figure out node between in and out segment
	auto find_node = [&] (Segment* in, Segment* out) {
		if (in->node_a == out->node_a || in->node_a == out->node_b)
			return in->node_a;
		else
			return in->node_b;
	};
	auto choose_lane = [&] (Segment* in, Node* node, Segment* out) {
		auto turn = classify_turn(node, in, out);
		auto dir = in->get_dir_to_node(node);

		for (auto& l : in->lanes) {
			if (  in->get_lane_layout(&l).direction == dir &&
				  (l.allowed_turns & turn) != 0) {
				return (int)(&l - in->lanes.data());
			}
		}

		assert(false);
		return 0;
	};
	auto choose_lane_for_building = [&] (Node* prev_node, Segment* in) {
		auto dir = in->get_dir_to_node(prev_node);
		// TODO: make it so lanes in a direction (sorted left to right) can be accessed easier
		for (auto& l : in->lanes) {
			if (in->get_lane_layout(&l).direction != dir) { // != for opposite direction
				return (int)(&l - in->lanes.data());
			}
		}
		return 0;
	};

	int i = idx > 0 ? (idx-1)/2 : 0;
	
	Segment* seg0 = i-1 >= 0      ? agent->path[i-1] : nullptr;
	Segment* seg1 = i   < num_seg ? agent->path[i  ] : nullptr;
	Segment* seg2 = i+1 < num_seg ? agent->path[i+1] : nullptr;
	Segment* seg3 = i+2 < num_seg ? agent->path[i+2] : nullptr;

	Node* node0 = seg0 && seg1 ? find_node(seg0, seg1) : nullptr;
	Node* node1 = seg1 && seg2 ? find_node(seg1, seg2) : nullptr;
	Node* node2 = seg2 && seg3 ? find_node(seg2, seg3) : nullptr;

	int lane1 = seg1 && seg2 ? choose_lane(seg1, node1, seg2) : -1;
	int lane2 = seg2 && seg3 ? choose_lane(seg2, node2, seg3) : -1;

	if (lane1 < 0) {
		assert(node0);
		lane1 = choose_lane_for_building(node0, seg1);
	}
	if (lane2 < 0 && node1) {
		lane2 = choose_lane_for_building(node1, seg2);
	}

	s.cur_lane  = { seg1, (uint16_t)lane1 };
	s.next_lane = { seg2, (uint16_t)lane2 };
	s.cur_node  = node1;

	if (s.cur_lane)  assert(s.cur_lane.lane  >= 0);
	if (s.next_lane) assert(s.next_lane.lane >= 0);

	if (idx == 0) {
		auto s_lane = s.cur_lane.clac_lane_info();
		float3 s0 = agent->start->pos;
		float3 s1 = (s_lane.a + s_lane.b) * 0.5f;

		s.state = AgentState::EXIT_BUILDING;
		s.next_start_t = 0.5f;

		s.next_agents = &s.cur_lane.agents().list;

		s.bezier = { s0, (s0+s1)*0.5f, s1 };
		s.pos_z = s0.z;
	}
	else if (idx == num_moves-1) {
		auto e_lane = s.cur_lane.clac_lane_info();
		float3 e0 = (e_lane.a + e_lane.b) * 0.5f;
		float3 e1 = agent->end->pos;

		s.state = AgentState::ENTER_BUILDING;

		s.bezier = { e0, (e0+e1)*0.5f, e1 };
		s.pos_z = e1.z;
	}
	else {
		auto l  = s.cur_lane.clac_lane_info();
		auto l2 = s.next_lane ? s.next_lane.clac_lane_info() : Line{0,0};

		if (s.cur_node) assert(contains(s.cur_node->in_lanes , s.cur_lane ));
		if (s.cur_node) assert(contains(s.cur_node->out_lanes, s.next_lane));

		if ((idx-1) % 2 == 0) {
			assert(s.cur_lane);

			s.state = AgentState::SEGMENT;

			s.cur_agents  = &s.cur_lane.agents().list;
			s.next_agents = s.cur_node ? &s.cur_node->agents.free : nullptr;


			// handle enter building
			if (!node1)
				s.end_t = 0.5f;

			s.bezier = { l.a, (l.a+l.b)*0.5f, l.b }; // 3d 2d?
			s.pos_z = l.a.z;
		}
		else {
			assert(s.cur_lane && s.cur_node && s.next_lane);

			if (agent->idx == idx)
				assert(contains(s.cur_node->agents.free.list, agent));

			s.state = AgentState::NODE;

			s.cur_agents  = &s.cur_node->agents.free;
			s.next_agents = &s.next_lane.agents().list;

			s.bezier = calc_curve(l, l2);
			s.pos_z = l.a.z;
		}
	}

	return s;
}

float _brake_for_dist (float obstacle_dist) {
	return clamp(map(obstacle_dist, 0.0f, 8.0f), 0.0f, 1.0f);
}
void brake_for_dist (Agent* agent, float obstacle_dist) {
	float brake = _brake_for_dist(obstacle_dist);
	agent->brake = min(agent->brake, brake);
}

void debug_node (App& app, Node* node, View3D const& view) {
	if (!node) return;

	g_dbgdraw.wire_circle(node->pos, node->_radius, lrgba(1,1,0,1));

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
			g_dbgdraw.wire_circle(agent.agent->center(), agent.agent->car_len()*0.5f, lrgba(1,0,0.5f,1));
	
			g_dbgdraw.text.draw_text(prints("%d%s", i++, agent.blocked ? " B":""),
				30, 1, g_dbgdraw.text.map_text(agent.agent->center(), view));
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
void debug_citizen (App& app, Citizen* cit, View3D const& view) {
	if (!cit || !cit->agent) return;

	float start_t = cit->agent->bez_t;
	for (int i=cit->agent->idx; ; ++i) {
		auto s = get_agent_state(cit->agent.get(), i);

		dbg_draw_bez(s.bezier, 0, 5, lrgba(1,1,0,1), start_t, s.end_t); 

		start_t = s.next_start_t;
		if (s.state == AgentState::ENTER_BUILDING) break;
	}

	ImGui::Separator();
	ImGui::TextColored(lrgba(cit->col, 1), "debug citizen");

	static ValuePlotter speed_plot = ValuePlotter();
	speed_plot.push_value(cit->agent->speed);
	speed_plot.imgui_display("speed", 0.0f, 100/KPH_PER_MS);

	if (cit->owned_car->name == "combi") {
		for (auto& m : _mats) {
			float3 center;
			float ang;
			cit->agent->calc_pos(&center, &ang);

			auto mat = translate(center) * rotate3_Z(ang) * inverse(m);

			auto pos = (float3)(mat * float4(0,0,0,1));
			g_dbgdraw.point(pos, 0.01f, lrgba(0,0,0,1));
			g_dbgdraw.line(pos, (float3)(mat * float4(.1f,0,0,1)), lrgba(1,0,0,1));
			g_dbgdraw.line(pos, (float3)(mat * float4(0,.1f,0,1)), lrgba(0,1,0,1));
			g_dbgdraw.line(pos, (float3)(mat * float4(0,0,.1f,1)), lrgba(0,0,1,1));
		}
	}
}

void dbg_brake_for (App& app, Agent* cur, float dist, float3 obstacle, lrgba col) {
	// dir does not actually point where we are going to stop
	// obsticle visualizes what object we are stopping for
	// dist is approx distance along bezier to stop at, which we don't bother visualizing exactly

	dist = max(dist, 0.0f);

	float3 pos = cur->front_pos;
	float3 dir = normalizesafe(obstacle - pos);
	float3 end = pos + float3(dir,0)*dist;
	float3 normal = float3(rotate90(dir), 0);

	g_dbgdraw.arrow(pos, obstacle - pos, 0.3f, col);
	g_dbgdraw.line(end - normal, end + normal, col);
}
void _FORCEINLINE dbg_brake_for_agent (App& app, Agent* cur, float dist, Agent* obstacle) {
	if (app.selection.get<Citizen*>() == cur->cit) {
		float3 center = (obstacle->rear_pos + obstacle->front_pos) * 0.5f;
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
		points[i+1] = bez.eval_value_fast_for_const_t(t);
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
			Agent* cur  = lane.list.list[i];
			
			// approx seperation using cur car bez_speed
			float dist = (prev->bez_t - cur->bez_t) * cur->bez_speed - (prev->car_len() + 1);

			brake_for_dist(cur, dist);
			dbg_brake_for_agent(app, cur, dist, prev);
		}
	}
}

NodeAgent* get_left_agent (NodeAgent& a, NodeAgent& b) {
	float2 dir_a = a.conn.pointsL[4] - a.conn.pointsL[0];
	float2 dir_b = b.conn.pointsL[4] - b.conn.pointsL[0];
	float d = dot(rotate90(dir_a), dir_b);

	return d > 0 ? &a : &b;
}
void _yield_for_car (App& app, Node* node, NodeAgent& a, NodeAgent& b, bool dbg) {
	// WARNING: a and b are kinda the wrong way around, b is on the left, ie. yielded for
	assert(a.agent != b.agent);
	
	auto conf = query_conflict(node, a.conn, b.conn);

	if (dbg) debug_conflict(a.conn, b.conn, conf);

	if (!conf)
		return;
	
	// TODO: calc this in query_conflict?
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

	float stop_k;
		
	// if same conn: follow car
	// if diverge:   follow car
	// if merge:     stop before conflict until b rear in conflict, then follow
	// if crossing:  stop before conflict
	if (same || diverge || (merge && b_rear_entered)) {
		// TODO: might want to do this differently if b_exited and we still follow
			
		// need to follow car, approx correct stop_k by mapping b rear from b's collision zone to a's zone
		stop_k = lerp(a_k0, a_k1, map(b.rear_k, b_k0, b_k1));
		stop_k -= SAFETY_DIST;
	}
	else {
		// TODO: calc this in query_conflict?
		float a_eta = (a_k0 - a.front_k) / (a.agent->speed + 1.0f);
		float b_eta = (b_k0 - b.front_k) / (b.agent->speed + 1.0f);

		// max .5m past stop line
		bool behind_stop_line = a.front_k < 0.5f;
		// wait before conflict if we reach it fast and yielded-for car arrives in similar time to us
		// otherwise wait at stop line
		bool need_wait = b_eta / a_eta > 3.0f || a_eta > 10.0f;

		if (behind_stop_line && need_wait) {
			// stop at stop line
			stop_k = -0.1f; // try to stop slight behind line to avoid (technically) standing in intersection?
		}
		else {
			// stop before conflict
			stop_k = a_k0;
			stop_k -= SAFETY_DIST;
		}
	}
	
	float dist = stop_k - a.front_k; // wait before conflict by default

	brake_for_dist(a.agent, dist);
	dbg_brake_for_agent(app, a.agent, dist, b.agent);

	// need to be careful because need to block cars that are behind other blocked cars or intersection can deadlock
	// but can't just block anyone who crosses path with blocked or we also deadlock
	// This seems to work
	if (b.blocked && (same || diverge) && !a_exited)
		a.blocked = true; // so swapping can let other go first if we are effectively blocked

}
bool swap_cars (App& app, Node* node, NodeAgent& a, NodeAgent& b, bool dbg, int b_idx) {
	assert(a.agent != b.agent);

	bool swap_valid = true;

	NodeAgent* left_agent = nullptr;

	auto conf = query_conflict(node, a.conn, b.conn);
	if (conf) {

		float a_k0 = conf.a_t0 * a.conn.bez_len;
		float a_k1 = conf.a_t1 * a.conn.bez_len;
		float b_k0 = conf.b_t0 * b.conn.bez_len;
		float b_k1 = conf.b_t1 * b.conn.bez_len;
		
		bool a_entered = a.front_k >= a_k0;
		bool a_exited  = a.rear_k  >= a_k1;
		bool b_entered = b.front_k >= b_k0;
		bool b_exited  = b.rear_k  >= b_k1;
		
		bool diverge = a.conn.conn.a == b.conn.conn.a; // same start point
		bool merge   = a.conn.conn.b == b.conn.conn.b; // same end point
		//bool crossing = !merge && !diverge; // normal crossing
		bool same = merge && diverge; // identical path

		// currently: b yielding for a  after swap: a yielding for b
		// if either exited, can swap (swap makes no difference)
		// if a_entered: a cant yield for b (because b would clip through a)
		// if diverge: cant swap unless either exited

		if (a_exited || b_exited) swap_valid = true;
		else if (a_entered)       swap_valid = false;
		else if (diverge)         swap_valid = false;
		else                      swap_valid = true;
		
		// detemine right before left
		bool can_yeild_rBl = !same && !diverge && !a_entered;
		bool same_yield_level = a.conn.conn.a.yield == b.conn.conn.a.yield;
		if (same_yield_level && can_yeild_rBl) {
			left_agent = get_left_agent(a, b);
		}
	}
	
	auto clac_penalty = [&] (NodeAgent& agent, float conf_t0) {
		auto& heur = app.net.settings.intersec_heur;

		float penalty = 0;

		// penalty for time to reach confict point if swapping with conflicting car
		if (conf) {
			float k0 = conf_t0 * agent.conn.bez_len;

			float conf_eta = (k0 - agent.front_k) / (agent.agent->speed + 1.0f);
			
			penalty += clamp(map(conf_eta, 1.0f, 6.0f), 0.0f, 1.0f) * heur.conflict_eta_penal;
		}

		if (&agent == left_agent) {
			penalty += heur.right_before_left_penal;
		}

		if (agent.conn.conn.a.yield)
			penalty += heur.yield_lane_penal;
		
		// eta to leave intersection
		float exit_eta = (agent.conn.bez_len - agent.front_k) / (agent.agent->speed + 1.0f);
		penalty += clamp(map(exit_eta, 1.0f, 6.0f), 0.0f, 1.0f) * heur.exit_eta_penal;

		// priority for progress through intersection
		// don't want distance from intersection to be a penalty, just to let cars in the intersection leave easier
		if (agent.front_k > 0) {
			float progress_ratio = agent.front_k / agent.conn.bez_len;
			penalty -= progress_ratio * heur.progress_boost;
		}

		// unbounded wait time priority, waiting cars will eventually be let through
		penalty -= agent.wait_time * heur.wait_boost_fac;
		
		return penalty;
	};
	
	bool do_swap = false;
	float a_penalty = clac_penalty(a, conf.a_t0);
	float b_penalty = clac_penalty(b, conf.b_t0);

	if (swap_valid) {
		if (a.blocked == b.blocked) {
			// sort by heuristic
			do_swap = a_penalty - b_penalty > 2;
			//do_swap = a_penalty > b_penalty;
		}
		else {
			// sort blocked last (stable sort because only between blocked/non-blocked)
			do_swap = a.blocked;
		}
	}

	if (dbg) {
		if (b_idx == 1) {
			ImGui::Text("Cars swap:");
			ImGui::TextColored(lrgba(a.agent->cit->col, 1), "#%02d", b_idx-1);
		}

		ImGui::Text("%7.3f%s", a_penalty, do_swap ? " S":"");
		ImGui::Text("%7.3f", b_penalty);

		ImGui::TextColored(lrgba(b.agent->cit->col, 1), "#%02d", b_idx);
	}

	return do_swap;
}

void update_node (App& app, Node* node, float dt) {
	bool node_dbg = app.selection.get<Node*>() == node;
	
	auto* sel  = app.selection .get<Citizen*>() ? app.selection .get<Citizen*>()->agent.get() : nullptr;
	auto* sel2 = app.selection2.get<Citizen*>() ? app.selection2.get<Citizen*>()->agent.get() : nullptr;
	
	auto dbg_avail_space = [&] (SegLane const& lane_out, Agent* a) {
		auto li = lane_out.clac_lane_info();
		
		auto pos = lerp(li.a, li.b, lane_out.agents().avail_space / lane_out.seg->_length);
		g_dbgdraw.point(pos, 1, lrgba(a->cit->col,1));
	};

	//
	for (auto& lane_out : node->out_lanes) {
		auto& avail_space = lane_out.agents().avail_space;
		avail_space = lane_out.seg->_length;
		
		for (auto* a : lane_out.seg->agents.lanes[lane_out.lane].list.list) {
			if (node_dbg) dbg_avail_space(lane_out, a);

			avail_space -= a->car_len() + SAFETY_DIST;
		}
	}
	
	// Track cars that are relevant to intersection
	for (auto& lane : node->in_lanes) {
		for (auto* agent : lane.agents().list.list) {
			if (node->agents.test.contains(agent)) continue; // TODO: Expensive contains with vector

			float dist = (1.0f - agent->bez_t) * agent->bez_speed;
			if (dist > 10.0f) break;

			//auto s = get_agent_state_only_conn(agent, agent->idx);
			if (!agent->state.cur_lane || !agent->state.next_lane)
				continue;

			NodeAgent a;
			a.agent = agent;
			a.node_idx = agent->idx+1;
			a.wait_time = 0;

			auto in_lane = node->get_in_lane(agent->state.cur_lane.seg, agent->state.cur_lane.lane);

			a.conn.conn = { in_lane, OutLane{ agent->state.next_lane } };
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
			
			if (dist > a.agent->car_len())
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
		
		a.rear_k = a.front_k - a.agent->car_len();
	};
	
	
	// allocate space in priority order and remember blocked cars
	for (auto& a : node->agents.test.list) {
		update_ks(a);
		
		a.blocked = false;
		a.wait_time += dt;

		if (a.agent->idx > a.node_idx) {
			// already in outgoing lane (don't need to wait and avoid counting avail space twice)
			continue;
		}

		// TODO: still reserve space even if none is avail if already on node?

		auto& avail_space = a.conn.conn.b.agents().avail_space;
		if (avail_space < a.agent->car_len()) {
			float dist = -a.front_k; // end of ingoing lane
			
			brake_for_dist(a.agent, dist);
			dbg_brake_for_blocked_lane(app, a, dist);

			a.blocked = true;
		}
		else {
			if (node_dbg) dbg_avail_space(a.conn.conn.b, a.agent);
			avail_space -= a.agent->car_len() + SAFETY_DIST;
		}
	}

	int count = (int)node->agents.test.list.size();
	
	//for (int i=0; i<count; ++i) {
	//	auto& b = node->agents.test.list[i];
	//
	//	// swap with car that has prio 1 higher according to heuristic
	//	if (i > 0) {
	//		auto& a = node->agents.test.list[i-1];
	//
	//		if (swap_cars(node, a, b)) {
	//			std::swap(a, b);
	//		}
	//	}
	//}

	// Check each car against higher prio cars to yield to them
	for (int i=0; i<count; ++i) {
		auto& a = node->agents.test.list[i];

		if (a.agent == sel || a.agent == sel2) {
			printf("");
		}
		
		//// swap with car that has prio 1 higher according to heuristic
		//if (i < count-1) {
		//	auto& b = node->agents.test.list[i+1];
		//
		//	if (swap_cars(node, a, b)) {
		//		std::swap(a, b);
		//	}
		//}

		// loop over all previous cars (higher prio to yield for)
		for (int j=0; j<i; ++j) {
			auto& b = node->agents.test.list[j];

			bool dbg = (a.agent == sel || a.agent == sel2) && (b.agent == sel || b.agent == sel2);
		
			if (dbg) {
				printf("");
			}

			_yield_for_car(app, node, a, b, dbg);
		}

		// brake for target lane car
		auto& target_lane = a.conn.conn.b.agents().list.list;
		if (!target_lane.empty() && target_lane.back() != a.agent) {
			float a_front_k = a.front_k - a.conn.bez_len; // relative to after node

			auto* b = target_lane.back();
			float b_rear_k = b->bez_t * b->bez_speed - b->car_len();

			float dist = b_rear_k - a_front_k;
			dist -= SAFETY_DIST;
		
			brake_for_dist(a.agent, dist);
			dbg_brake_for_agent(app, a.agent, dist, b);
		}
	}

	for (int i=1; i<count; ++i) {
		auto& a = node->agents.test.list[i-1];
		auto& b = node->agents.test.list[i];
	
		// swap with car that has prio 1 higher according to heuristic
		if (swap_cars(app, node, a, b, node_dbg, i) && dt > 0) { // HACK: dt>0 for debugging
			std::swap(a, b);
		}
	}
}

float calc_car_drag (float cur_speed) {
	float drag_fac = 0.0014f; // ~220km/h top speed
	
	return drag_fac * cur_speed*cur_speed;
}
float calc_car_accel (float base_accel, float target_speed, float cur_speed) {
	float accel = base_accel;
	
	//// slow down acceleration close to target_speed
	//float target_ease = 0.2f;
	//float end_slow_amount = 0.3f;
	//
	//if (cur_speed > target_speed * (1.0f - target_ease)) {
	//	float fac = map(cur_speed, target_speed * (1.0f - target_ease), target_speed);
	//	accel *= fac*fac;
	//}
	//
	//// slow down acceleration from standstill
	//float start_slow_end = 2.5f;
	//float start_slow_amount = 0.3f;
	//if (cur_speed < start_slow_end) {
	//	accel *= lerp(start_slow_amount, 1, cur_speed / start_slow_end);
	//}
	
	accel -= calc_car_drag(cur_speed);
	return accel;
}
float calc_car_deccel (float base_deccel, float target_speed, float cur_speed) {
	float deccel = base_deccel;

	// TODO: include brake distance target here to break more smoothly?

	deccel += calc_car_drag(cur_speed);

	return deccel;
}

float get_cur_speed_limit (Agent* agent) {
	auto state = agent->state.state;
	if (state == AgentState::SEGMENT) {
		return agent->state.cur_lane.seg->asset->speed_limit;
	}
	else if (state == AgentState::NODE) {
		float a = agent->state.cur_lane .seg->asset->speed_limit;
		float b = agent->state.next_lane.seg->asset->speed_limit;
		return min(a, b); // TODO: ??
	}
	else {
		return 20 / KPH_PER_MS;
	}
}

float network::Agent::car_len () {
	return cit->owned_car->mesh.aabb.size().x;
}
void network::Agent::calc_pos (float3* pos, float* ang) {
	float3 dir = front_pos - rear_pos;
	*pos = front_pos - normalizesafe(dir) * car_len()*0.5f;
	*ang = angle2d((float2)dir);
}

void update_vehicle_suspension (App& app, Agent& agent, float2 local_accel, float dt) {
	// assume constant mass

	float2 ang = agent.suspension_ang;
	float2 vel = agent.suspension_ang_vel;

	// spring resitive accel
	//float2 accel = -ang * app.net.settings.suspension_spring_k;
	
	// quadratic for more smooth spring limit (and more wobbly around zero)
	float2 accel = -ang * abs(ang / app.net.settings.suspension_max_ang) * app.net.settings.suspension_spring_k * 3;
	
	// spring point accel
	accel += local_accel * app.net.settings.suspension_accel_fac;
	// spring dampening
	accel -= vel * app.net.settings.suspension_spring_damp;

	// apply vel, pos and clamp
	vel += accel * dt;
	ang += vel * dt;
	ang = clamp(ang, -app.net.settings.suspension_max_ang,
	                 +app.net.settings.suspension_max_ang);

	agent.suspension_ang = ang;
	agent.suspension_ang_vel = vel;
}

void update_vehicle (App& app, Metrics::Var& met, Agent* agent, float dt) {
	if (app.sim_paused)
		return;

	assert(agent->bez_t < 1.0f);
	float speed_limit = get_cur_speed_limit(agent);

	float old_speed = agent->speed;
	float new_speed = old_speed;

	// car speed change
	float target_speed = speed_limit * agent->brake;
	if (target_speed >= new_speed) {
		float accel = calc_car_accel(app.net.settings.car_accel, speed_limit, new_speed);
		new_speed += accel * dt;
		new_speed = min(new_speed, target_speed);
	}
	else {
		//new_speed = target_speed; // brake instantly for now
		new_speed -= calc_car_deccel(app.net.settings.car_deccel, speed_limit, new_speed);
		new_speed = max(new_speed, target_speed);
	}

	agent->speed = new_speed;
	met.total_flow += agent->speed / speed_limit;
	
	// move car with speed on bezier based on previous frame delta t
	agent->bez_t += agent->speed * dt / agent->bez_speed;

	// do bookkeeping when car reaches end of current bezier
	if (agent->bez_t >= agent->state.end_t) {
		agent->idx++;
		agent->bez_t = agent->state.next_start_t;
		assert(agent->bez_t >= 0 && agent->bez_t < 1);

		if (agent->state.cur_agents)  agent->state.cur_agents ->remove(agent);
		if (agent->state.next_agents) agent->state.next_agents->add(agent);

		if (agent->state.state == AgentState::ENTER_BUILDING) {
			// end path
			agent->cit->target_building = agent->end;
			agent->cit->agent = nullptr;
			return;
		}

		agent->state = get_agent_state(agent, agent->idx);
	}

	// eval bezier at car front
	auto bez_res = agent->state.bezier.eval_with_curv(agent->bez_t);
	// remember bezier delta t for next frame
	agent->bez_speed = length(bez_res.vel); // bezier t / delta pos

	agent->turn_curv = bez_res.curv; // TODO: to be correct for wheel turning this would need to be computed based on the rear axle

	// actually move car rear using (bogus) trailer formula
	float2 new_front = bez_res.pos;
	
	float2 old_front = (float2)agent->front_pos;
	float2 old_rear  = (float2)agent->rear_pos;

	float2 forw = normalizesafe(old_front - old_rear);
	float2 right = -rotate90(forw);
	//float forw_amount = dot(new_front - old_front, forw);

	float car_len = agent->car_len();
	float2 ref_point = old_rear + car_len*app.net.settings.car_rear_drag_ratio * forw; // Kinda works to avoid goofy car rear movement?

	float2 new_rear = new_front - normalizesafe(new_front - ref_point) * car_len;

	agent->front_pos = float3(new_front, agent->state.pos_z);
	agent->rear_pos  = float3(new_rear,  agent->state.pos_z);
	
	{
		float2 old_center = (old_front + old_rear) * 0.5f;
		float2 new_center = (new_front + new_rear) * 0.5f;
		float2 center_vel = (new_center - old_center) / dt;
		float2 center_accel = (center_vel - float2(agent->vel)) / dt;

		if (agent->cit == app.selection.get<Citizen*>()) {
			g_dbgdraw.arrow(float3(new_front, agent->state.pos_z), float3(center_vel, 0), 0.2f, lrgba(0,0,1,1));
			g_dbgdraw.arrow(float3(new_front, agent->state.pos_z), float3(center_accel*0.1f, 0), 0.2f, lrgba(0,1,0,1));
		}

		// accel from world to local space
		//float accel_cap = 30; // we get artefacts with huge accelerations due to discontinuities, cap accel to hide
		//center_accel.y = clamp( dot(center_accel, right), -accel_cap, accel_cap);
		//center_accel.x = clamp( dot(center_accel, forw ), -accel_cap, accel_cap);
		center_accel.x = dot(center_accel, right);
		center_accel.y = dot(center_accel, forw );
		update_vehicle_suspension(app, *agent, -center_accel, dt);
		
		if (agent->cit == app.selection.get<Citizen*>()) {
			//printf("%7.3f %7.3f  |  %7.3f %7.3f\n", center_accel.x, center_accel.y, center_vel.x, center_vel.y);
			
			float2 a = right * agent->suspension_ang.x + forw * agent->suspension_ang.y;
			g_dbgdraw.point(float3(new_center, agent->state.pos_z), 0.3f, lrgba(.5f,.5f,.1f,0.5f));
			g_dbgdraw.point(float3(new_center + a*10, agent->state.pos_z), 0.3f, lrgba(1,1,0.5f,1));
			
			float turn_r = 1.0f/agent->turn_curv;
			g_dbgdraw.wire_circle(float3(new_rear - right * turn_r, agent->state.pos_z), turn_r, lrgba(1,0,0,1), 128);
		}

		agent->vel = float3(center_vel, 0);
	}
}

void Metrics::update (Var& var, App& app) {
	avg_flow = var.total_flow / (float)app.entities.citizens.size();

	flow_plot.push_value(avg_flow);
}

void Network::simulate (App& app) {
	ZoneScoped;

	pathing_count = 0;

	float dt = app.sim_paused ? 0 : app.input.dt * app.sim_speed;
	app._test_time += dt;

	auto do_pathfind = [&] (Citizen* cit) {
		auto* cur_target = app.entities.buildings[ app.test_rand.uniformi(0, (int)app.entities.buildings.size()) ].get();
		
		assert(cit->target_building->connected_segment);
		if (cit->target_building->connected_segment) {
			ZoneScopedN("pathfind");

			pathing_count++;

			auto agent = std::make_unique<network::Agent>();
			agent->cit = cit;
			agent->start = cit->target_building;
			agent->end   = cur_target;
			bool valid = pathfind(cit->target_building->connected_segment, cur_target->connected_segment, agent.get());
			if (valid) {
				cit->target_building = nullptr;
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
		Metrics::Var met;
		
		{
			ZoneScopedN("init pass");
			for (auto& cit : app.entities.citizens) {
				if (cit->target_building) continue;
				cit->agent->brake = 1;
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
				update_node(app, node.get(), dt);
			}
		}
		
		{
			ZoneScopedN("final pass");
			for (auto& cit : app.entities.citizens) {
				if (cit->target_building) {
					do_pathfind(cit.get());
				}
				else {
					update_vehicle(app, met, cit->agent.get(), dt);
				}
			}
		}

		metrics.update(met, app);
	//}

	static RunningAverage pathings_avg(30);
	pathings_avg.push((float)pathing_count);
	float min, max;
	float avg = pathings_avg.calc_avg(&min, &max);
	ImGui::Text("pathing_count: avg %3.1f min: %3.1f max: %3.1f", avg, min, max);
	

	ImGui::Text("nodes: %05d segments: %05d citizens: %05d",
		(int)nodes.size(), (int)segments.size(), (int)app.entities.citizens.size());
	
	ImGui::Text("last dijkstra: iter: %05d iter_dupl: %05d iter_lanes: %05d", _dijk_iter, _dijk_iter_dupl, _dijk_iter_lanes);

}

void Network::draw_debug (App& app, View3D& view) {
	debug_node(app, app.selection.get<Node*>(), view);
	debug_citizen(app, app.selection.get<Citizen*>(), view);
}

} // namespace network
