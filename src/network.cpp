#include "common.hpp"
#include "network.hpp"
#include "app.hpp"

namespace network {

// TODO: remove app as dependency? What is it actually needed for?

// Pathfinding ignores lanes other than checking if any lane allows the turn to a node being visited
// Note: lane selection happens later during car path follwing, a few segments into the future
bool Network::pathfind (Segment* start, Segment* target, ActiveVehicle* vehicle) {
	ZoneScoped;
	// use dijkstra

	// TODO: why did I create an optimization and then not use it? it it working? did I want to defer to when I can have more features?
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

	// prepare all nodes
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
		Turns allowed = Turns::NONE;
		for (auto& lane : cur_node->_pred_seg->lanes) {
			allowed |= lane.allowed_turns;
		}

		// update neighbours with new minimum cost
		for (auto& seg : cur_node->segments) {
			for (auto lane : seg->in_lanes(cur_node)) {
				Node* other_node = seg->get_other_node(cur_node);

				// check if turn to this node is actually allowed
				auto turn = classify_turn(cur_node, cur_node->_pred_seg, lane.seg);
				if ((allowed & turn) == Turns::NONE) {
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
		vehicle->path.push_back(reverse_segments[i]);
	}

	return true;
}

PathState get_path_state (Network& net, ActiveVehicle* vehicle, int idx, PathState* prev_state = nullptr) {
	PathState s = {};

	int num_seg = (int)vehicle->path.size();
	int num_moves = num_seg + (num_seg-1) + 2;
	assert(num_seg >= 1);

	auto seeded_rand = Random(hash(idx, (uint64_t)vehicle));

	// find next (non-straight) turn in next N nodes
	// choose lane for segment before that
	// backiterate and apply 'target' lane to each by 'staying on' lane backwards for M steps
	// possibly have random

	// figure out node between in and out segment
	auto find_node = [&] (Segment* in, Segment* out) {
		if (in->node_a == out->node_a || in->node_a == out->node_b)
			return in->node_a;
		else
			return in->node_b;
	};
	
	auto roll_random_lane_switch = [&] () {
		return seeded_rand.chance(net._random_lane_switching_chance);
	};
	auto pick_random_allowed_lane = [&] (Segment::LanesRange& lanes, Turns turn, SegLane default_lane, bool exclude_default=false) {
		auto choose = [&] (SegLane lane) {
			return (lane.get().allowed_turns & turn) != Turns::NONE &&
				(exclude_default ? lane != default_lane : true);
		};
		
		int count = 0;
		for (auto lane : lanes) {
			if (choose(lane))
				count++;
		}
		
		if (count > 0) {
			int choice = seeded_rand.uniformi(0, count);
			int idx = 0;
			for (auto lane : lanes) {
				if (choose(lane) && idx++ == choice)
					return lane;
			}
		}
		// if no lanes were chosen for random pick either because exclude_default or missing allowed turn (pathfinding bug?)
		return default_lane;
	};
	auto pick_stay_in_lane = [&] (Segment::LanesRange& next_lanes, SegLane& cur_lane) {
		// stupid logic for now
		auto lane = (laneid_t)clamp(cur_lane.lane, next_lanes.first, next_lanes.end_-1);
		return SegLane{ next_lanes.seg, lane };
	};

	auto pick_next_lane = [&] (SegLane& cur_lane, Segment* in, Node* node, Segment* out) {
		auto turn = classify_turn(node, in, out);
		auto in_lanes = in->in_lanes(node);
		
		SegLane stay_lane = pick_stay_in_lane(in_lanes, cur_lane);
		bool can_stay_in_lane = (stay_lane.get().allowed_turns & turn) != Turns::NONE;

		// if we can't stay in lane, pick random turn lane
		if (!can_stay_in_lane)
			return pick_random_allowed_lane(in_lanes, turn, stay_lane);

		// randomly switch lanes, but never switch
		if (roll_random_lane_switch())
			return pick_random_allowed_lane(in_lanes, turn, stay_lane, true);

		// otherwise stay in lane if allowed
		return stay_lane;
	};

	if (idx == 0) {
		Segment* start_seg = vehicle->path[0];
		Node* next_node = num_seg > 1 ? find_node(start_seg, vehicle->path[1]) : nullptr;
		LaneDir seg_dir = start_seg->get_dir_to_node(next_node); // null is ok

		s.cur_lane = {};
		// currently wrong, since it ignored turn arrows if next move is turn, need to somehow use same logic as in PathState::SEGMENT;
		s.next_lane = start_seg->lanes_in_dir(seg_dir).outer(); // start on outermost lane
		
		auto s_lane = s.next_lane.clac_lane_info();
		float3 s0 = vehicle->start->pos;
		float3 s1 = (s_lane.a + s_lane.b) * 0.5f;

		s.state = PathState::EXIT_BUILDING;
		s.next_start_t = 0.5f;

		s.next_vehicles = &s.next_lane.vehicles().list;

		s.bezier = { s0, (s0+s1)*0.5f, s1 };
		s.pos_z = s0.z;
	}
	else if (idx == num_moves-1) {
		assert(prev_state);
		auto prev_lane = prev_state->cur_lane;
		s.cur_lane = {};
		s.next_lane = {};

		auto e_lane = prev_lane.clac_lane_info();
		float3 e0 = (e_lane.a + e_lane.b) * 0.5f;
		float3 e1 = vehicle->end->pos;

		s.state = PathState::ENTER_BUILDING;

		s.bezier = { e0, (e0+e1)*0.5f, e1 };
		s.pos_z = e1.z;
	}
	else {
		Node* cur_node;
		if ((idx-1) % 2 == 0) {
			{
				assert(prev_state);
				s.cur_lane = prev_state->next_lane;
			
				int i = (idx-1)/2;
				
				Segment* seg0 = vehicle->path[i]; // current segment
				Segment* seg1 = i+1 < num_seg ? vehicle->path[i+1] : nullptr; // next segment (after current node)
				Segment* seg2 = i+2 < num_seg ? vehicle->path[i+2] : nullptr; // segment after that (after next node)
		
					  cur_node  = seg1 ? find_node(seg0, seg1) : nullptr; // current/next segment -> current node
				Node* next_node = seg2 ? find_node(seg1, seg2) : nullptr; // next/after that segment -> next node
		
				if (!seg1) {
					// already on target lane (end of trip)
					s.next_lane = {};
				}
				else if (!seg2) {
					assert(cur_node && seg1);
					// lane after node is target lane, can't pick normally
					LaneDir dir = seg1->get_dir_from_node(cur_node);
					s.next_lane = seg1->lanes_in_dir(dir).outer(); // end on outermost lane
				}
				else {
					assert(seg1 && next_node && seg2);
					// still has next node, pick lane after cur_node based on required lane for turn at next_node
					s.next_lane = pick_next_lane(s.cur_lane, seg1, next_node, seg2);
				}
			}

			auto l = s.cur_lane.clac_lane_info();
		
			s.state = PathState::SEGMENT;
		
			s.cur_vehicles  = &s.cur_lane.vehicles().list;
			s.next_vehicles = cur_node ? &cur_node->vehicles.free : nullptr;
			
			// handle enter building
			if (!cur_node)
				s.end_t = 0.5f;
		
			s.bezier = { l.a, (l.a+l.b)*0.5f, l.b }; // 3d 2d?
			s.pos_z = l.a.z;
		}
		else {
			s.cur_lane = prev_state->cur_lane;
			s.next_lane = prev_state->next_lane;
			cur_node = find_node(s.cur_lane.seg, s.next_lane.seg);

			auto l  = s.cur_lane.clac_lane_info();
			auto l2 = s.next_lane ? s.next_lane.clac_lane_info() : Line{0,0};

			//if (s.cur_node) assert(contains(s.cur_node->in_lanes , s.cur_lane ));
			//if (s.cur_node) assert(contains(s.cur_node->out_lanes, s.next_lane));

			assert(s.cur_lane && cur_node && s.next_lane);
		
			if (vehicle->idx == idx)
				assert(contains(cur_node->vehicles.free.list, vehicle));
		
			s.state = PathState::NODE;
		
			s.cur_vehicles  = &cur_node->vehicles.free;
			s.next_vehicles = &s.next_lane.vehicles().list;
		
			s.bezier = calc_curve(l, l2);
			s.pos_z = l.a.z;
		}
	}

	return s;
}

float _brake_for_dist (float obstacle_dist) {
	return clamp(map(obstacle_dist, 0.0f, 8.0f), 0.0f, 1.0f);
}
void brake_for_dist (ActiveVehicle* vehicle, float obstacle_dist) {
	float brake = _brake_for_dist(obstacle_dist);
	vehicle->brake = min(vehicle->brake, brake);
}

void debug_node (App& app, Node* node, View3D const& view) {
	if (!node) return;

	g_dbgdraw.wire_circle(node->pos, node->_radius, lrgba(1,1,0,1));

	//{
	//	int i=0;
	//	for (auto& v : node->vehicles.free.list) {
	//		g_dbgdraw.wire_circle(v->cit->center(), CAR_SIZE*0.5f, lrgba(1,0,0.5f,1));
	//
	//		g_dbgdraw.text.draw_text(prints("%d", i++),
	//			30, 1, g_dbgdraw.text.map_text(v->cit->center(), app.view));
	//	}
	//}
	{
		int i=0;
		for (auto& v : node->vehicles.test.list) {
			g_dbgdraw.wire_circle(v.vehicle->center(), v.vehicle->car_len()*0.5f, lrgba(1,0,0.5f,1));
	
			g_dbgdraw.text.draw_text(prints("%d%s", i++, v.blocked ? " B":""),
				30, 1, g_dbgdraw.text.map_text(v.vehicle->center(), view));
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

	ImGui::Text("%d conflicts cached", (int)node->vehicles.conflict_cache.size());

	//{
	//	Hashmap<SegLane, lrgba, SegLaneHasher> cols;
	//	int col_i = 0;
	//	auto new_col = [&] () -> lrgba {
	//		return g_dbgdraw.COLS[col_i++ % ARRLEN(g_dbgdraw.COLS)];
	//	};
	//
	//	for (auto& kv : node->vehicles.conflict_cache) {
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
void debug_person (App& app, Person* person, View3D const& view) {
	if (!person || !person->vehicle) return;

	{
		PathState s = person->vehicle->state; // copy
		float start_t = person->vehicle->bez_t;

		for (int i=person->vehicle->idx; ; ++i) {
			dbg_draw_bez(s.bezier, 0, 5, lrgba(1,1,0,1), start_t, s.end_t); 

			start_t = s.next_start_t;
			if (s.state == PathState::ENTER_BUILDING) break;

			s = get_path_state(app.net, person->vehicle.get(), i+1, &s);
		}
	}

	ImGui::Separator();
	ImGui::TextColored(lrgba(person->col, 1), "debug person");
	
	ImGui::Text("Speed Limit: %7s", format_speed(get_cur_speed_limit(person->vehicle.get()), app.settings.speed_unit).c_str());
	ImGui::Text("Speed: %7s",       format_speed(person->vehicle->speed, app.settings.speed_unit).c_str());

	static ValuePlotter speed_plot = ValuePlotter();
	speed_plot.push_value(person->vehicle->speed);
	speed_plot.imgui_display("speed", 0.0f, 100/KPH_PER_MS);

	for (auto& bone : person->owned_vehicle->bone_mats) {
		float3 center;
		float ang;
		person->vehicle->calc_pos(&center, &ang);

		auto mat = translate(center) * rotate3_Z(ang) * bone.bone2mesh;

		auto pos = (float3)(mat * float4(0,0,0,1));
		g_dbgdraw.point(pos, 0.01f, lrgba(0,0,0,1));
		g_dbgdraw.line(pos, (float3)(mat * float4(.1f,0,0,1)), lrgba(1,0,0,1));
		g_dbgdraw.line(pos, (float3)(mat * float4(0,.1f,0,1)), lrgba(0,1,0,1));
		g_dbgdraw.line(pos, (float3)(mat * float4(0,0,.1f,1)), lrgba(0,0,1,1));
	}
}

void dbg_brake_for (App& app, ActiveVehicle* cur, float dist, float3 obstacle, lrgba col) {
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
void _FORCEINLINE dbg_brake_for_vehicle (App& app, ActiveVehicle* cur, float dist, ActiveVehicle* obstacle) {
	if (app.interact.selection.get<Person*>() == cur->cit) {
		float3 center = (obstacle->rear_pos + obstacle->front_pos) * 0.5f;
		dbg_brake_for(app, cur, dist, center, lrgba(1,0.1f,0,1));
	}
}
void _FORCEINLINE dbg_brake_for_blocked_lane (App& app, NodeVehicle& v, float dist) {
	if (app.interact.selection.get<Person*>() == v.vehicle->cit) {
		dbg_brake_for(app, v.vehicle, dist, v.conn.conn.b.clac_lane_info().a, lrgba(0.2f,0.8f,1,1));
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

	auto conf = node->vehicles.conflict_cache.get_or_create(key, [&] () -> Conflict {
		return check_conflict(*pa, *pb); // cache ordered conflict
	});

	// reverse ordering for result, effectively reusing b->a conflict as a->b
	return order ? conf : Conflict{ conf.b_t0, conf.b_t1, conf.a_t0, conf.a_t1 };
}

void update_segment (App& app, Segment* seg) {
	for (auto& lane : seg->vehicles.lanes) {
		// brake for car in front
		for (int i=1; i<(int)lane.list.list.size(); ++i) {
			ActiveVehicle* prev = lane.list.list[i-1];
			ActiveVehicle* cur  = lane.list.list[i];
			
			// approx seperation using cur car bez_speed
			float dist = (prev->bez_t - cur->bez_t) * cur->bez_speed - (prev->car_len() + 1);

			brake_for_dist(cur, dist);
			dbg_brake_for_vehicle(app, cur, dist, prev);
		}
	}
}

NodeVehicle* get_left_vehicle (NodeVehicle& a, NodeVehicle& b) {
	float2 dir_a = a.conn.pointsL[4] - a.conn.pointsL[0];
	float2 dir_b = b.conn.pointsL[4] - b.conn.pointsL[0];
	float d = dot(rotate90(dir_a), dir_b);

	return d > 0 ? &a : &b;
}
void _yield_for_car (App& app, Node* node, NodeVehicle& a, NodeVehicle& b, bool dbg) {
	// WARNING: a and b are kinda the wrong way around, b is on the left, ie. yielded for
	assert(a.vehicle != b.vehicle);
	
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
		float a_eta = (a_k0 - a.front_k) / (a.vehicle->speed + 1.0f);
		float b_eta = (b_k0 - b.front_k) / (b.vehicle->speed + 1.0f);

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

	brake_for_dist(a.vehicle, dist);
	dbg_brake_for_vehicle(app, a.vehicle, dist, b.vehicle);

	// need to be careful because need to block cars that are behind other blocked cars or intersection can deadlock
	// but can't just block anyone who crosses path with blocked or we also deadlock
	// This seems to work
	if (b.blocked && (same || diverge) && !a_exited)
		a.blocked = true; // so swapping can let other go first if we are effectively blocked

}
bool swap_cars (App& app, Node* node, NodeVehicle& a, NodeVehicle& b, bool dbg, int b_idx) {
	assert(a.vehicle != b.vehicle);

	bool swap_valid = true;

	NodeVehicle* left_vehicle = nullptr;

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
		bool same_yield_level = a.conn.conn.a.get().yield == b.conn.conn.a.get().yield;
		if (same_yield_level && can_yeild_rBl) {
			left_vehicle = get_left_vehicle(a, b);
		}
	}
	
	auto clac_penalty = [&] (NodeVehicle& v, float conf_t0) {
		auto& heur = app.net.settings.intersec_heur;

		float penalty = 0;

		// penalty for time to reach confict point if swapping with conflicting car
		if (conf) {
			float k0 = conf_t0 * v.conn.bez_len;

			float conf_eta = (k0 - v.front_k) / (v.vehicle->speed + 1.0f);
			
			penalty += clamp(map(conf_eta, 1.0f, 6.0f), 0.0f, 1.0f) * heur.conflict_eta_penal;
		}

		if (&v == left_vehicle) {
			penalty += heur.right_before_left_penal;
		}

		if (v.conn.conn.a.get().yield)
			penalty += heur.yield_lane_penal;
		
		// eta to leave intersection
		float exit_eta = (v.conn.bez_len - v.front_k) / (v.vehicle->speed + 1.0f);
		penalty += clamp(map(exit_eta, 1.0f, 6.0f), 0.0f, 1.0f) * heur.exit_eta_penal;

		// priority for progress through intersection
		// don't want distance from intersection to be a penalty, just to let cars in the intersection leave easier
		if (v.front_k > 0) {
			float progress_ratio = v.front_k / v.conn.bez_len;
			penalty -= progress_ratio * heur.progress_boost;
		}

		// unbounded wait time priority, waiting cars will eventually be let through
		penalty -= v.wait_time * heur.wait_boost_fac;
		
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
			ImGui::TextColored(lrgba(a.vehicle->cit->col, 1), "#%02d", b_idx-1);
		}

		ImGui::Text("%7.3f%s", a_penalty, do_swap ? " S":"");
		ImGui::Text("%7.3f", b_penalty);

		ImGui::TextColored(lrgba(b.vehicle->cit->col, 1), "#%02d", b_idx);
	}

	return do_swap;
}

void update_node (App& app, Node* node, float dt) {
	bool node_dbg = app.interact.selection.get<Node*>() == node;

	if (node->traffic_light) {
		assert(node->traffic_light->behavior);
		node->traffic_light->behavior->update(node, dt);
	}

	auto* sel  = app.interact.selection.get<Person*>() ? app.interact.selection.get<Person*>()->vehicle.get() : nullptr;
	auto* sel2 = app.interact.hover    .get<Person*>() ? app.interact.hover    .get<Person*>()->vehicle.get() : nullptr;
	
	auto dbg_avail_space = [&] (SegLane const& lane_out, ActiveVehicle* a) {
		auto li = lane_out.clac_lane_info();
		
		auto pos = lerp(li.a, li.b, lane_out.vehicles().avail_space / lane_out.seg->_length);
		g_dbgdraw.point(pos, 1, lrgba(a->cit->col,1));
	};

	//
	for (auto& seg : node->segments) {
		for (auto lane_out : seg->out_lanes(node)) {
			auto& avail_space = lane_out.vehicles().avail_space;
			avail_space = lane_out.seg->_length;
		
			for (auto* a : lane_out.seg->vehicles.lanes[lane_out.lane].list.list) {
				if (node_dbg) dbg_avail_space(lane_out, a);

				avail_space -= a->car_len() + SAFETY_DIST;
			}
		}
	}
	
	// Track cars that are relevant to intersection
	for (auto& seg : node->segments) {
		for (auto lane : seg->in_lanes(node)) {
			for (auto* v : lane.vehicles().list.list) {
				if (node->vehicles.test.contains(v)) continue; // TODO: Expensive contains with vector

				float dist = (1.0f - v->bez_t) * v->bez_speed;
				if (dist > 10.0f) break;

				//auto s = get_vehicle_state_only_conn(v, v->idx);
				if (!v->state.cur_lane || !v->state.next_lane)
					continue;

				NodeVehicle vehicle;
				vehicle.vehicle = v;
				vehicle.node_idx = v->idx+1;
				vehicle.wait_time = 0;

				vehicle.conn.conn = { v->state.cur_lane, v->state.next_lane };
				auto bez = calc_curve(vehicle.conn.conn.a.clac_lane_info(), vehicle.conn.conn.b.clac_lane_info());
				vehicle.conn.bez_len = bez.approx_len(COLLISION_STEPS);
			
				_gen_points(vehicle.conn.pointsL, vehicle.conn.conn, -LANE_COLLISION_R);
				_gen_points(vehicle.conn.pointsR, vehicle.conn.conn, +LANE_COLLISION_R);

				node->vehicles.test.add(vehicle);
			}
		}
	}
	node->vehicles.test.remove_if([&] (NodeVehicle& v) {
		if (v.vehicle->idx > v.node_idx+1)
			return true; // past outgoing lane
		if (v.vehicle->idx == v.node_idx+1) {
			// in outgoing lane
			float dist = v.vehicle->bez_t * v.vehicle->bez_speed;
			
			if (dist > v.vehicle->car_len())
				return true;
		}
		return false;
	});
	
	auto update_ks = [&] (NodeVehicle& v) {

		// ingoing lane
		if (v.vehicle->idx == v.node_idx-1) {
			// extrapolate and map from negative to 0
			v.front_k = (v.vehicle->bez_t - 1.0f) * v.vehicle->bez_speed;
		}
		// on node
		else if (v.vehicle->idx == v.node_idx) {
			// approximate by just mapping t (which is wrong)
			v.front_k = v.vehicle->bez_t * v.conn.bez_len;
		}
		// outgoing lane
		else {
			assert(v.vehicle->idx == v.node_idx+1);
			// extrapolate and map from negative to 0
			v.front_k = v.vehicle->bez_t * v.vehicle->bez_speed + v.conn.bez_len;
		}
		
		v.rear_k = v.front_k - v.vehicle->car_len();
	};
	
	
	// allocate space in priority order and remember blocked cars
	for (auto& v : node->vehicles.test.list) {
		update_ks(v);
		
		v.blocked = false;
		v.wait_time += dt;

		if (v.vehicle->idx > v.node_idx) {
			// already in outgoing lane (don't need to wait and avoid counting avail space twice)
			continue;
		}

		// TODO: still reserve space even if none is avail if already on node?

		auto& avail_space = v.conn.conn.b.vehicles().avail_space;
		if (avail_space < v.vehicle->car_len()) {
			float dist = -v.front_k; // end of ingoing lane
			
			brake_for_dist(v.vehicle, dist);
			dbg_brake_for_blocked_lane(app, v, dist);

			v.blocked = true;
		}
		else {
			if (node_dbg) dbg_avail_space(v.conn.conn.b, v.vehicle);
			avail_space -= v.vehicle->car_len() + SAFETY_DIST;
		}
	}

	int count = (int)node->vehicles.test.list.size();
	
	//for (int i=0; i<count; ++i) {
	//	auto& b = node->vehicles.test.list[i];
	//
	//	// swap with car that has prio 1 higher according to heuristic
	//	if (i > 0) {
	//		auto& a = node->vehicles.test.list[i-1];
	//
	//		if (swap_cars(node, a, b)) {
	//			std::swap(a, b);
	//		}
	//	}
	//}

	// Check each car against higher prio cars to yield to them
	for (int i=0; i<count; ++i) {
		auto& a = node->vehicles.test.list[i];

		//if (a.vehicle == sel || a.vehicle == sel2) {
		//	printf("");
		//}
		
		//// swap with car that has prio 1 higher according to heuristic
		//if (i < count-1) {
		//	auto& b = node->vehicles.test.list[i+1];
		//
		//	if (swap_cars(node, a, b)) {
		//		std::swap(a, b);
		//	}
		//}

		// loop over all previous cars (higher prio to yield for)
		for (int j=0; j<i; ++j) {
			auto& b = node->vehicles.test.list[j];

			bool dbg = (a.vehicle == sel || a.vehicle == sel2) && (b.vehicle == sel || b.vehicle == sel2);
		
			//if (dbg) {
			//	printf("");
			//}

			_yield_for_car(app, node, a, b, dbg);
		}

		// brake for target lane car
		auto& target_lane = a.conn.conn.b.vehicles().list.list;
		if (!target_lane.empty() && target_lane.back() != a.vehicle) {
			float a_front_k = a.front_k - a.conn.bez_len; // relative to after node

			auto* b = target_lane.back();
			float b_rear_k = b->bez_t * b->bez_speed - b->car_len();

			float dist = b_rear_k - a_front_k;
			dist -= SAFETY_DIST;
		
			brake_for_dist(a.vehicle, dist);
			dbg_brake_for_vehicle(app, a.vehicle, dist, b);
		}
	}

	for (int i=1; i<count; ++i) {
		auto& a = node->vehicles.test.list[i-1];
		auto& b = node->vehicles.test.list[i];
	
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

float network::ActiveVehicle::car_len () {
	return cit->owned_vehicle->mesh.aabb.size().x;
}
void network::ActiveVehicle::calc_pos (float3* pos, float* ang) {
	float3 dir = front_pos - rear_pos;
	*pos = front_pos - normalizesafe(dir) * car_len()*0.5f;
	*ang = angle2d((float2)dir);
}

void update_vehicle_suspension (App& app, ActiveVehicle& vehicle, float2 local_accel, float dt) {
	// assume constant mass

	float2 ang = vehicle.suspension_ang;
	float2 vel = vehicle.suspension_ang_vel;

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
	vel = clamp(vel, -100, +100);

	ang += vel * dt;
	ang = clamp(ang, -app.net.settings.suspension_max_ang,
	                 +app.net.settings.suspension_max_ang);

	vehicle.suspension_ang = ang;
	vehicle.suspension_ang_vel = vel;
}

void update_vehicle (App& app, Metrics::Var& met, ActiveVehicle* vehicle, float dt) {

	float aggress = vehicle->cit->topspeed_accel_mul();

	assert(vehicle->bez_t < 1.0f);
	float speed_limit = aggress * get_cur_speed_limit(vehicle);

	float old_speed = vehicle->speed;
	float new_speed = old_speed;

	// car speed change
	float target_speed = speed_limit * vehicle->brake;
	if (target_speed >= new_speed) {
		float accel = aggress * calc_car_accel(app.net.settings.car_accel, speed_limit, new_speed);
		new_speed += accel * dt;
		new_speed = min(new_speed, target_speed);
	}
	else {
		//new_speed = target_speed; // brake instantly for now
		new_speed -= aggress * calc_car_deccel(app.net.settings.car_deccel, speed_limit, new_speed);
		new_speed = max(new_speed, target_speed);
	}

	vehicle->speed = new_speed;
	met.total_flow += vehicle->speed / speed_limit;
	
	// move car with speed on bezier based on previous frame delta t
	float delta_dist = vehicle->speed * dt;
	vehicle->bez_t += delta_dist / vehicle->bez_speed;

	// do bookkeeping when car reaches end of current bezier
	if (vehicle->bez_t >= vehicle->state.end_t) {
		vehicle->idx++;
		vehicle->bez_t = vehicle->state.next_start_t;
		assert(vehicle->bez_t >= 0 && vehicle->bez_t < 1);

		if (vehicle->state.cur_vehicles)  vehicle->state.cur_vehicles ->remove(vehicle);
		if (vehicle->state.next_vehicles) vehicle->state.next_vehicles->add(vehicle);

		if (vehicle->state.state == PathState::ENTER_BUILDING) {
			// end path
			vehicle->cit->cur_building = vehicle->end;
			vehicle->cit->vehicle = nullptr; // ugh, but works
			return;
		}

		vehicle->state = get_path_state(app.net, vehicle, vehicle->idx, &vehicle->state);
	}

	// eval bezier at car front
	auto bez_res = vehicle->state.bezier.eval_with_curv(vehicle->bez_t);
	// remember bezier delta t for next frame
	vehicle->bez_speed = length(bez_res.vel); // bezier t / delta pos
	float2 bez_dir = bez_res.vel / vehicle->bez_speed;

	// actually move car rear using (bogus) trailer formula
	float2 new_front = bez_res.pos;
	
	float2 old_front = (float2)vehicle->front_pos;
	float2 old_rear  = (float2)vehicle->rear_pos;

	float2 forw = normalizesafe(old_front - old_rear);
	float2 right = -rotate90(forw);
	//float forw_amount = dot(new_front - old_front, forw);

	float car_len = vehicle->car_len();
	float2 ref_point = old_rear + car_len*app.net.settings.car_rear_drag_ratio * forw; // Kinda works to avoid goofy car rear movement?

	float2 new_rear = new_front - normalizesafe(new_front - ref_point) * car_len;

	vehicle->front_pos = float3(new_front, vehicle->state.pos_z);
	vehicle->rear_pos  = float3(new_rear,  vehicle->state.pos_z);
	
	// totally wack with car_rear_drag_ratio
	vehicle->turn_curv = bez_res.curv; // TODO: to be correct for wheel turning this would need to be computed based on the rear axle

	{
		float2 old_center = (old_front + old_rear) * 0.5f;
		float2 new_center = (new_front + new_rear) * 0.5f;
		float2 center_vel   = dt == 0 ? 0 : (new_center - old_center) / dt;
		float2 center_accel = dt == 0 ? 0 : (center_vel - float2(vehicle->center_vel)) / dt;

		if (vehicle->cit == app.interact.selection.get<Person*>()) {
			g_dbgdraw.arrow(float3(new_front, vehicle->state.pos_z), float3(center_vel, 0), 0.2f, lrgba(0,0,1,1));
			g_dbgdraw.arrow(float3(new_front, vehicle->state.pos_z), float3(center_accel*0.1f, 0), 0.2f, lrgba(0,1,0,1));
		}

		// accel from world to local space
		//float accel_cap = 30; // we get artefacts with huge accelerations due to discontinuities, cap accel to hide
		//center_accel.y = clamp( dot(center_accel, right), -accel_cap, accel_cap);
		//center_accel.x = clamp( dot(center_accel, forw ), -accel_cap, accel_cap);
		center_accel.x = dot(center_accel, right);
		center_accel.y = dot(center_accel, forw );
		update_vehicle_suspension(app, *vehicle, -center_accel, dt);
		
		if (vehicle->cit == app.interact.selection.get<Person*>()) {
			//printf("%7.3f %7.3f  |  %7.3f %7.3f\n", center_accel.x, center_accel.y, center_vel.x, center_vel.y);
			
			float2 a = right * vehicle->suspension_ang.x + forw * vehicle->suspension_ang.y;
			g_dbgdraw.point(float3(new_center, vehicle->state.pos_z), 0.1f, lrgba(.5f,.5f,.1f,0.5f));
			g_dbgdraw.point(float3(new_center + a*10, vehicle->state.pos_z), 0.1f, lrgba(1,1,0.5f,1));
			
			float turn_r = 1.0f/vehicle->turn_curv;
			g_dbgdraw.wire_circle(float3(new_rear - right * turn_r, vehicle->state.pos_z), turn_r, lrgba(1,0,0,1), 128);
		}

		vehicle->center_vel = float3(center_vel, 0);

		float wheel_circum = vehicle->cit->owned_vehicle->wheel_r * (2*PI);

		vehicle->wheel_roll += delta_dist / wheel_circum;
		vehicle->wheel_roll = fmodf(vehicle->wheel_roll, 1.0f);
	}
}

void Metrics::update (Var& var, App& app) {
	avg_flow = var.total_flow / (float)app.entities.persons.size();

	flow_plot.push_value(avg_flow);
}

void Network::simulate (App& app) {
	ZoneScoped;

	pathing_count = 0;

	float dt = app.sim_dt();
	app._test_time += dt;

	auto start_trip = [&] (Person* person) {
		auto* target = app.entities.buildings[ app.sim_rand.uniformi(0, (int)app.entities.buildings.size()) ].get();
		
		assert(person->cur_building->connected_segment);
		if (person->cur_building->connected_segment) {
			ZoneScopedN("pathfind");

			pathing_count++;

			auto vehicle = std::make_unique<network::ActiveVehicle>();
			vehicle->cit = person;
			vehicle->start = person->cur_building;
			vehicle->end   = target;

			bool valid = pathfind(person->cur_building->connected_segment, target->connected_segment, vehicle.get());
			if (valid) {
				person->cur_building = nullptr;
				person->vehicle = std::move(vehicle);
				// get initial state
				person->vehicle->state = get_path_state(*this, person->vehicle.get(), person->vehicle->idx);
				return true;
			}
		}
		return false;
	};
	
	// to avoid debugging overlays only showing while not paused, only skip moving the car when paused, later actually skip sim steps
	//if (!app.sim_paused) {
		Metrics::Var met;
		
		{ // TODO: only iterate active vehicles
			ZoneScopedN("init pass");
			for (auto& cit : app.entities.persons) {
				if (cit->cur_building) continue;
				cit->vehicle->brake = 1;
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
			for (auto& person : app.entities.persons) {
				if (person->cur_building) {
					person->stay_timer += dt;
					if (person->stay_timer >= 3.0f) {
						person->stay_timer = 0;

						if (start_trip(person.get())) {
							update_vehicle(app, met, person->vehicle.get(), 0); // 0 dt timestep to init some values properly
						}
					}
				}
				else {
					assert(person->vehicle);
					update_vehicle(app, met, person->vehicle.get(), dt);
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
	

	ImGui::Text("nodes: %05d segments: %05d persons: %05d",
		(int)nodes.size(), (int)segments.size(), (int)app.entities.persons.size());
	
	ImGui::Text("last dijkstra: iter: %05d iter_dupl: %05d iter_lanes: %05d", _dijk_iter, _dijk_iter_dupl, _dijk_iter_lanes);

}

void Network::draw_debug (App& app, View3D& view) {
	debug_node(app, app.interact.selection.get<Node*>(), view);
	debug_person(app, app.interact.hover.get<Person*>(), view);
}

} // namespace network
