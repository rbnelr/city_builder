#include "common.hpp"
#include "network.hpp"
#include "app.hpp"

namespace network {

// TODO: remove app as dependency? What is it actually needed for?
	
//// Pathfinding

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
				if (!any_set(allowed, turn)) {
					// turn not allowed
					//assert(false); // currently impossible, only the case for roads with no right turn etc.
					continue;
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

////
PathState get_path_state (Network& net, ActiveVehicle* vehicle, int idx, PathState* prev_state = nullptr) {
	PathState s = {};

	int num_seg = (int)vehicle->path.size();
	int num_moves = num_seg + (num_seg-1) + 2;
	assert(num_seg >= 1);

	// Make lane selection deterministic for path visualization
	// TODO: this requirement might go away once I do lane selections more robustly
	//   Might still want to keep determinism based on car id + path progress int, just so reloading a save state results in the same behavior?
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
			return any_set(lane.get().allowed_turns, turn) &&
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
		bool can_stay_in_lane = any_set(stay_lane.get().allowed_turns, turn);

		// if we can't stay in lane, pick random turn lane
		if (!can_stay_in_lane)
			return pick_random_allowed_lane(in_lanes, turn, stay_lane);

		// randomly switch lanes, but never switch
		if (roll_random_lane_switch())
			return pick_random_allowed_lane(in_lanes, turn, stay_lane, true);

		// otherwise stay in lane if allowed
		return stay_lane;
	};

	auto building_enter_bezier = [&] (SegLane& lane, Building* build, float* out_t) {
		auto lane_bez = lane._bezier();
		float len = lane_bez.approx_len(4);
		float t = max(0.5f - 5 / len, 0.0f); // curve such that the endpoint is 5m earlier on the lane than the middle point

		float3 ctrl = lane_bez.eval(0.5f).pos;
		float3 end  = lane_bez.eval(t).pos;

		*out_t = t;
		return Bezier3(end, ctrl, ctrl, build->pos);
	};
	auto building_exit_bezier = [&] (SegLane& lane, Building* build, float* out_t) {
		auto lane_bez = lane._bezier();
		float len = lane_bez.approx_len(4);
		float t = min(0.5f + 5 / len, 1.0f);

		float3 ctrl = lane_bez.eval(0.5f).pos;
		float3 end  = lane_bez.eval(t).pos;

		*out_t = t;
		return Bezier3(build->pos, ctrl, ctrl, end);
	};

	if (idx == 0) {
		s.state = PathState::EXIT_BUILDING;

		Segment* start_seg = vehicle->path[0];
		Node* next_node = num_seg > 1 ? find_node(start_seg, vehicle->path[1]) : nullptr;
		LaneDir seg_dir = start_seg->get_dir_to_node(next_node); // null is ok

		s.cur_lane = {};
		// currently wrong, since it ignored turn arrows if next move is turn, need to somehow use same logic as in PathState::SEGMENT;
		s.next_lane = start_seg->lanes_in_dir(seg_dir).outer(); // start on outermost lane
		
		s.next_vehicles = &s.next_lane.vehicles().list;
		s.bezier = building_exit_bezier(s.next_lane, vehicle->start, &s.next_start_t);
	}
	else if (idx == num_moves-1) {
		s.state = PathState::ENTER_BUILDING;

		assert(prev_state);
		auto prev_lane = prev_state->cur_lane;
		s.cur_lane = {};
		s.next_lane = {};

		float t;
		s.bezier = building_enter_bezier(prev_lane, vehicle->end, &t);
	}
	else {
		if ((idx-1) % 2 == 0) {
			s.state = PathState::SEGMENT;
			{
				assert(prev_state);
				s.cur_lane = prev_state->next_lane;
			
				int i = (idx-1)/2;
				
				Segment* seg0 = vehicle->path[i]; // current segment
				Segment* seg1 = i+1 < num_seg ? vehicle->path[i+1] : nullptr; // next segment (after current node)
				Segment* seg2 = i+2 < num_seg ? vehicle->path[i+2] : nullptr; // segment after that (after next node)
		
				s.cur_node      = seg1 ? find_node(seg0, seg1) : nullptr; // current/next segment -> current node
				Node* next_node = seg2 ? find_node(seg1, seg2) : nullptr; // next/after that segment -> next node
		
				if (!seg1) {
					// already on target lane (end of trip)
					s.next_lane = {};
				}
				else if (!seg2) {
					assert(s.cur_node && seg1);
					// lane after node is target lane, can't pick normally
					LaneDir dir = seg1->get_dir_from_node(s.cur_node);
					s.next_lane = seg1->lanes_in_dir(dir).outer(); // end on outermost lane
				}
				else {
					assert(seg1 && next_node && seg2);
					// still has next node, pick lane after cur_node based on required lane for turn at next_node
					s.next_lane = pick_next_lane(s.cur_lane, seg1, next_node, seg2);
				}

				if (seg1) {
					assert(s.cur_node);
					s.cur_turn = classify_turn(s.cur_node, seg0, seg1);
				}
			}
		
			s.cur_vehicles  = &s.cur_lane.vehicles().list;
			s.next_vehicles = s.cur_node ? &s.cur_node->vehicles.free : nullptr;
			
			// handle enter building lane end_t
			if (!s.cur_node) {
				building_enter_bezier(prev_state->cur_lane, vehicle->end, &s.end_t);
			}
			
			s.bezier = s.cur_lane._bezier();
		}
		else {
			s.state = PathState::NODE;

			s.cur_lane = prev_state->cur_lane;
			s.next_lane = prev_state->next_lane;
			s.cur_turn = prev_state->cur_turn;
			s.cur_node = find_node(s.cur_lane.seg, s.next_lane.seg);

			//if (s.cur_node) assert(contains(s.cur_node->in_lanes , s.cur_lane ));
			//if (s.cur_node) assert(contains(s.cur_node->out_lanes, s.next_lane));

			assert(s.cur_lane && s.cur_node && s.next_lane);
		
			if (vehicle->idx == idx)
				assert(contains(s.cur_node->vehicles.free.list, vehicle));
		
			s.cur_vehicles  = &s.cur_node->vehicles.free;
			s.next_vehicles = &s.next_lane.vehicles().list;
			
			s.bezier = s.cur_node->calc_curve(s.cur_lane, s.next_lane);
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

//// Visualization
void overlay_lane_vehicle (App& app, ActiveVehicle* vehicle, lrgba col, int tex) {
	// TODO: simple extrapolation for now, abstract this away and roll this into the logic used by node traffic sim
	// to have one place where covered bezier ranges are determined
	// (enable asking for car front and back bezier t, coresponding beziers (and ones inbetween if car longer than 1 segment/node etc.)
	// sample bezier with t to determine worldspace pos, or draw bezier with overlay etc.
	
	Bezier3 cur_bez = vehicle->state.bezier;
	float t1 = vehicle->bez_t;
	float t0 = vehicle->bez_t - vehicle->car_len() / vehicle->bez_speed; // t - len / (dlen / dt) => t - dt
	if (t0 < 0) {
		// car rear on different bezier!
		t0 = 0;

		// TODO: actually enable computing previous bezier!
	}
	
	app.overlay.draw_bezier_portion(cur_bez, float2(t0,t1), float2(LANE_COLLISION_R*2, 1), col, tex);
}

void dbg_node_lane_alloc (App& app, Node* node) {
	
	auto dbg_avail_space = [&] (SegLane& lane_out, ActiveVehicle* a) {
		// interpolate lane bezier instead of trying to go call overlay_lane_vehicle on vehicle
		// because vehicle is not actually in this spot yet! remember we allocate lane space by placing virtual cars!
		
		auto bez = lane_out._bezier();
		float t1 = lane_out.vehicles().avail_space / lane_out.seg->_length;

		float bez_speed = length(bez.eval(t1).vel);
		float t0 = t1 - a->car_len() / bez_speed; // TODO: cache accurate k in vehicle and eliminate this calculation!
		
		app.overlay.draw_bezier_portion(bez, float2(t0,t1), float2(LANE_COLLISION_R*2, 1), lrgba(a->cit->col, 0.8f), OverlayDraw::PATTERN_STRIPED);
	};

	for (auto& seg : node->segments) {
		for (auto lane_out : seg->out_lanes(node)) {
			auto& avail_space = lane_out.vehicles().avail_space;

			avail_space = lane_out.seg->_length;
			for (auto* a : lane_out.seg->vehicles.lanes[lane_out.lane].list.list) {
				dbg_avail_space(lane_out, a);
				avail_space -= a->car_len() + SAFETY_DIST;
			}
		}
	}
	
	// allocate space in priority order and remember blocked cars
	for (auto& v : node->vehicles.test.list) {
		if (v.vehicle->idx > v.node_idx) {
			// already in outgoing lane
			continue;
		}

		auto& avail_space = v.conn.conn.b.vehicles().avail_space;
		if (avail_space >= v.vehicle->car_len()) {
			dbg_avail_space(v.conn.conn.b, v.vehicle);
			avail_space -= v.vehicle->car_len() + SAFETY_DIST;
		}
	}
}
bool dbg_conflicts (App& app, Node* node, ActiveVehicle* vehicle);

void debug_node (App& app, Node* node, View3D const& view) {
	if (!node) return;
	
	static bool debug_priority_order = false;
	static bool debug_node_lane_alloc = false;
	static bool show_lane_connections = false;
	if (imgui_Header("debug_node", true)) {
		ImGui::Checkbox("debug_priority_order", &debug_priority_order);
		ImGui::Checkbox("debug_node_lane_alloc", &debug_node_lane_alloc);
		ImGui::Checkbox("show_lane_connections", &show_lane_connections);

		ImGui::Text("%d conflicts cached", (int)node->vehicles.conflict_cache.size());

		ImGui::PopID();
	}

	g_dbgdraw.wire_circle(node->pos, node->_radius, lrgba(1,1,0,1));

	if (debug_priority_order) {
		int i=0;
		for (auto& v : node->vehicles.test.list) {
			//overlay_lane_vehicle(app, v.vehicle, lrgba(v.vehicle->cit->col, 0.5f), OverlayDraw::PATTERN_SOLID);
	
			g_dbgdraw.text.draw_text(prints("%d%s", i++, v.blocked ? " B":""),
				30, 1, g_dbgdraw.text.map_text(v.vehicle->center(), view));
		}
	}

	if (debug_node_lane_alloc)
		dbg_node_lane_alloc(app, node);

	if (show_lane_connections) {
		
		int idx = 0;
		for (auto* seg : node->segments) {
			auto col = render::SimpleColors::get(idx++);
			col.w = 0.5f;
			for (auto lane_in : seg->in_lanes(node)) {
				for (auto lane_out : lane_in.get().connections) {
					auto bez = node->calc_curve(lane_in, lane_out);
					app.overlay.draw_bezier_portion(bez, float2(0,1), float2(3.5f, 1), col, OverlayDraw::TEXTURE_THIN_ARROW);
				}
			}
		}
	}

#if 0
	{ // visualize conficts somehow? idk this code is old
		Hashmap<SegLane, lrgba, SegLaneHasher> cols;
		int col_i = 0;
		auto new_col = [&] () -> lrgba {
			return g_dbgdraw.COLS[col_i++ % ARRLEN(g_dbgdraw.COLS)];
		};
	
		for (auto& kv : node->vehicles.conflict_cache) {
			auto col0 = cols.get_or_create(kv.first.a.a, new_col);
			auto col1 = cols.get_or_create(kv.first.a.b, new_col);
			auto col2 = cols.get_or_create(kv.first.b.a, new_col);
			auto col3 = cols.get_or_create(kv.first.b.b, new_col);
		
			ImGui::TextColored(col0, "%p:%d",	kv.first.a.a.seg, kv.first.a.a.lane); ImGui::SameLine();
			ImGui::TextColored(col1, "-%p:%d",	kv.first.a.b.seg, kv.first.a.b.lane); ImGui::SameLine();
			ImGui::Text(" | "); ImGui::SameLine();
			ImGui::TextColored(col2, "%p:%d",	kv.first.b.a.seg, kv.first.b.a.lane); ImGui::SameLine();
			ImGui::TextColored(col3, "-%p:%d",	kv.first.b.b.seg, kv.first.b.b.lane);
		}
		
		for (auto& kv : cols) {
			auto p = kv.first.clac_lane_info();
			g_dbgdraw.line(p.a, p.b, kv.second);
		}
	}
#endif
}
void debug_person (App& app, Person* person, View3D const& view) {
	if (!person || !person->vehicle) return;
	
	static bool visualize_lane_section = false;
	static bool visualize_path = true;
	static bool visualize_conflicts = true;
	static bool visualize_bones = false;
	static ValuePlotter speed_plot = ValuePlotter();
	speed_plot.push_value(person->vehicle->speed);

	if (imgui_Header("debug_person", true)) {
		ImGui::Checkbox("visualize_lane_section", &visualize_lane_section);
		ImGui::Checkbox("visualize_path", &visualize_path);
		ImGui::Checkbox("visualize_conflicts", &visualize_conflicts);
		ImGui::Checkbox("visualize_bones", &visualize_bones);
		
		ImGui::Separator();
		ImGui::TextColored(lrgba(person->col, 1), "debug person");
	
		ImGui::Text("Speed Limit: %7s", app.options.format_speed(get_cur_speed_limit(person->vehicle.get())).c_str());
		ImGui::Text("Speed: %7s",       app.options.format_speed(person->vehicle->speed).c_str());

		speed_plot.imgui_display("speed", 0.0f, 100/KPH_PER_MS);

		ImGui::PopID();
	}
	
	if (visualize_lane_section)
		overlay_lane_vehicle(app, person->vehicle.get(), lrgba(person->col, 1), OverlayDraw::PATTERN_SOLID);
	
	bool did_viz_conflicts = false;
	if (visualize_conflicts && person->vehicle->state.cur_node) {
		did_viz_conflicts = dbg_conflicts(app, person->vehicle->state.cur_node, person->vehicle.get());
	}

	if (visualize_path) {
		PathState s = person->vehicle->state; // copy
		float start_t = person->vehicle->bez_t;

		bool skip_next_node = did_viz_conflicts; // skip first arrow if already drawing arrow on intersection
		
		for (int i=person->vehicle->idx; ; ++i) {
			if (skip_next_node && s.state == PathState::NODE) {
				skip_next_node = false;
			}
			else {
				app.overlay.draw_bezier_portion(s.bezier, float2(start_t, s.end_t),
					float2(2, 1), lrgba(1,1,0,0.75f), OverlayDraw::TEXTURE_THIN_ARROW);
			}

			start_t = s.next_start_t;
			if (s.state == PathState::ENTER_BUILDING) break;

			s = get_path_state(app.network, person->vehicle.get(), i+1, &s);
		}
	}

	if (visualize_bones) {
		for (auto& bone : person->owned_vehicle->bone_mats) {
			float3 center;
			float ang;
			person->vehicle->calc_pos(&center, &ang);

			auto mat = translate(center) * rotate3_Z(ang) * bone.bone2mesh;

			auto pos = (float3)(mat * float4(0,0,0,1));
			//g_dbgdraw.point(pos, 0.01f, lrgba(0,0,0,1));
			g_dbgdraw.line(pos, (float3)(mat * float4(.25f,0,0,1)), lrgba(1,0,0,1));
			g_dbgdraw.line(pos, (float3)(mat * float4(0,.25f,0,1)), lrgba(0,1,0,1));
			g_dbgdraw.line(pos, (float3)(mat * float4(0,0,.25f,1)), lrgba(0,0,1,1));
		}
	}
}

#if 0
void dbg_brake_for (App& app, ActiveVehicle* cur, float dist, float3 obstacle, lrgba col) {
	// dir does not actually point where we are going to stop
	// obsticle visualizes what object we are stopping for
	// dist is approx distance along bezier to stop at, which we don't bother visualizing exactly

	dist = max(dist, 0.0f);

	float3 pos = cur->front_pos;
	float3 dir = normalizesafe(obstacle - pos);
	float3 end = pos + dir*dist;
	float3 normal = relative2dir(dir).right;

	g_dbgdraw.arrow(pos, obstacle - pos, 0.3f, col);
	g_dbgdraw.line(end - normal, end + normal, col);
}
#else
void dbg_brake_for (App& app, ActiveVehicle* cur, float dist, float3 obstacle, lrgba col) {}
#endif
void _FORCEINLINE dbg_brake_for_vehicle (App& app, ActiveVehicle* cur, float dist, ActiveVehicle* obstacle) {
	if (app.interact.selection.get<Person*>() == cur->cit) {
		float3 center = (obstacle->rear_pos + obstacle->front_pos) * 0.5f;
		dbg_brake_for(app, cur, dist, center, lrgba(1,0.1f,0,1));
	}
}
void _FORCEINLINE dbg_brake_for_blocked_lane_start (App& app, NodeVehicle& v, float dist, SegLane& lane) {
	if (app.interact.selection.get<Person*>() == v.vehicle->cit) {
		dbg_brake_for(app, v.vehicle, dist, lane._bezier().a, lrgba(0.2f,0.8f,1,1));
	}
}
void _FORCEINLINE dbg_brake_for_blocked_lane_end (App& app, NodeVehicle& v, float dist, SegLane& lane) {
	if (app.interact.selection.get<Person*>() == v.vehicle->cit) {
		dbg_brake_for(app, v.vehicle, dist, lane._bezier().b, lrgba(0.2f,0.8f,1,1));
	}
}

//// Segment logic
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

//// Vehicle logic
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

	if (a.conn.a == b.conn.a) { // same start point, code miss intersection, force it
		u0 = 0; v0 = 0;
	}
	if (a.conn.b == b.conn.b) { // same end   point, code miss intersection, force it
		u1 = 1; v1 = 1;
	}

	return { u0, u1, v0, v1 };
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

bool dbg_conflicts (App& app, Node* node, ActiveVehicle* vehicle) {
	int idx = kiss::indexof(node->vehicles.test.list, vehicle, [&] (NodeVehicle const& l, ActiveVehicle* r) { return l.vehicle == r; });
	if (idx < 0)
		return false;
	float2 sz = float2(LANE_COLLISION_R*2, 1);
	auto& a = node->vehicles.test.list[idx];

	auto has_conflict = [] (Node* node, NodeVehicle& a, NodeVehicle& b, Conflict& conf) {
		conf = query_conflict(node, a.conn, b.conn);
		if (!conf) return false;
		
		// Need to keep this code in sync with yield_for_car! (which sucks, not sure what the alternative is if I want to seperate the vis code this cleanly)
		float a_k1 = conf.a_t1 * a.conn.bez_len;
		float b_k1 = conf.b_t1 * b.conn.bez_len;
		
		bool a_exited  = a.rear_k  >= a_k1;
		bool b_exited  = b.rear_k  >= b_k1;
		
		bool diverge = a.conn.conn.a == b.conn.conn.a; // same start point
		bool merge   = a.conn.conn.b == b.conn.conn.b; // same end point
		//bool crossing = !merge && !diverge; // normal crossing
		bool same = merge && diverge; // identical path
	
		if (a_exited || b_exited || diverge)
			return false;
		return true;
	};

	// visualized vehicle's path through node as thick yellow arrow
	app.overlay.draw_bezier_portion(a.conn.bezier, float2(0,1), sz, lrgba(0.9f,0.9f,0.1f, 0.9f), OverlayDraw::TEXTURE_THICK_ARROW);
	
	// loop over all previous cars (higher prio to yield for)
	for (int j=0; j<idx; ++j) {
		auto& b = node->vehicles.test.list[j];

		Conflict conf;
		if (!has_conflict(node, a, b, conf)) continue;
		
		// conflict on visualized vehicle's path as red striped zone
		app.overlay.draw_bezier_portion(a.conn.bezier, float2(conf.a_t0, conf.a_t1), sz, lrgba(1.0f,0.02f,0.02f, 1), OverlayDraw::PATTERN_STRIPED);
	}

	// draw red arrows last to overlap all striped regions (looks nicer)
	for (int j=0; j<idx; ++j) {
		auto& b = node->vehicles.test.list[j];

		Conflict conf;
		if (!has_conflict(node, a, b, conf)) continue;
		
		// yielded-to vehicle's path through node as thin red arrow
		app.overlay.draw_bezier_portion(b.conn.bezier, float2(0,1), sz, lrgba(1.0f,0.08f,0.08f, 0.9f), OverlayDraw::TEXTURE_THIN_ARROW);
	}

	return true;
}

void debug_conflict (CachedConnection const& a, CachedConnection const& b, Conflict& conf) {
	for (int i=0; i<COLLISION_STEPS; ++i) {
		g_dbgdraw.line(float3(a.pointsL[i], ROAD_Z), float3(a.pointsL[i+1], ROAD_Z), lrgba(1,1,0,1));
		g_dbgdraw.line(float3(a.pointsR[i], ROAD_Z), float3(a.pointsR[i+1], ROAD_Z), lrgba(1,1,0,1));
		g_dbgdraw.line(float3(b.pointsL[i], ROAD_Z), float3(b.pointsL[i+1], ROAD_Z), lrgba(0,1,1,1));
		g_dbgdraw.line(float3(b.pointsR[i], ROAD_Z), float3(b.pointsR[i+1], ROAD_Z), lrgba(0,1,1,1));
	}

	auto draw_line = [&] (float2 const* L, float2 const* R, float t) {
		int i = (int)(t * COLLISION_STEPS);
		t = t * COLLISION_STEPS - i;

		g_dbgdraw.line(
			float3(lerp(L[i], L[i+1], t), ROAD_Z),
			float3(lerp(R[i], R[i+1], t), ROAD_Z), lrgba(1,0,0,1));
	};
	if (conf) {
		draw_line(a.pointsL, a.pointsR, conf.a_t0);
		draw_line(a.pointsL, a.pointsR, conf.a_t1);
		draw_line(b.pointsL, b.pointsR, conf.b_t0);
		draw_line(b.pointsL, b.pointsR, conf.b_t1);
	}
}

NodeVehicle* get_left_vehicle (NodeVehicle& a, NodeVehicle& b) {
	float2 dir_a = a.conn.pointsL[4] - a.conn.pointsL[0];
	float2 dir_b = b.conn.pointsL[4] - b.conn.pointsL[0];
	float d = dot(rotate90(dir_a), dir_b);

	return d > 0 ? &a : &b;
}
void yield_for_car (App& app, Node* node, NodeVehicle& a, NodeVehicle& b, bool dbg) {
	// WARNING: a and b are kinda the wrong way around, b is on the left, ie. yielded for
	assert(a.vehicle != b.vehicle);
	
	auto conf = query_conflict(node, a.conn, b.conn);
	
	//if (dbg) debug_conflict(a.conn, b.conn, conf);
	
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
		bool behind_stop_line = app.network.settings.intersec_heur.avoid_blocking_intersection &&
			a.front_k < 0.5f;
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
	
	float dist = stop_k - a.front_k;

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
		auto& heur = app.network.settings.intersec_heur;

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

#if 0
	if (dbg) {
		if (b_idx == 1) {
			ImGui::Text("Cars swap:");
			ImGui::TextColored(lrgba(a.vehicle->cit->col, 1), "#%02d", b_idx-1);
		}

		ImGui::Text("%7.3f%s", a_penalty, do_swap ? " S":"");
		ImGui::Text("%7.3f", b_penalty);

		ImGui::TextColored(lrgba(b.vehicle->cit->col, 1), "#%02d", b_idx);
	}
#endif

	return do_swap;
}

void cache_conn (Node* node, CachedConnection& conn, ActiveVehicle* vehicle) {
	conn.conn = { vehicle->state.cur_lane, vehicle->state.next_lane };
	conn.bezier = node->calc_curve(conn.conn.a, conn.conn.b);
	conn.bez_len = conn.bezier.approx_len(COLLISION_STEPS);
	
	auto bez2d = (Bezier2)conn.bezier;
	bez2d.calc_points(conn.pointsL, COLLISION_STEPS+1, -LANE_COLLISION_R);
	bez2d.calc_points(conn.pointsR, COLLISION_STEPS+1, +LANE_COLLISION_R);
}

void update_node (App& app, Node* node, float dt) {
	bool node_dbg = app.interact.selection.get<Node*>() == node;

	auto* sel  = app.interact.selection.get<Person*>() ? app.interact.selection.get<Person*>()->vehicle.get() : nullptr;
	auto* sel2 = app.interact.hover    .get<Person*>() ? app.interact.hover    .get<Person*>()->vehicle.get() : nullptr;

	if (node->traffic_light) {
		assert(node->traffic_light);
		node->traffic_light->update(node, dt);
	}
	
	//
	for (auto& seg : node->segments) {
		for (auto lane_out : seg->out_lanes(node)) {
			auto& avail_space = lane_out.vehicles().avail_space;

			avail_space = lane_out.seg->_length;
			for (auto* a : lane_out.seg->vehicles.lanes[lane_out.lane].list.list) {
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
				cache_conn(node, vehicle.conn, v);

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

		if (v.vehicle->idx < v.node_idx && node->traffic_light) {
			// still in front of intersection, respect traffic lights
			auto in_lane = v.conn.conn.a;

			auto cur_phase = node->traffic_light->decode_phase();
			auto signal_slot = node->traffic_light->_find_signal_slot(node, in_lane);
			auto lane_signal = node->traffic_light->get_signal(cur_phase, signal_slot);

			if (lane_signal == TrafficSignalState::RED) {
				// anything other than red means GO
				float dist = -v.front_k; // end of ingoing lane
				brake_for_dist(v.vehicle, dist);
				dbg_brake_for_blocked_lane_end(app, v, dist, in_lane);

				v.blocked = true;
			}
		}

		auto& avail_space = v.conn.conn.b.vehicles().avail_space;

		bool space_left = avail_space >= v.vehicle->car_len();
		bool already_on_node = v.vehicle->idx >= v.node_idx;

		// reserve space either if already on node or if on incoming lane and not blocked by traffic light
		if (already_on_node || (space_left && !v.blocked)) {
			// still reserve space even if none is avail if already on node
			avail_space -= v.vehicle->car_len() + SAFETY_DIST;
		}
		else {
			float dist = -v.front_k; // end of ingoing lane
			brake_for_dist(v.vehicle, dist);
			dbg_brake_for_blocked_lane_start(app, v, dist, v.conn.conn.b);

			v.blocked = true;
		}
	}

	int count = (int)node->vehicles.test.list.size();
	
	// Check each car against higher prio cars to yield to them
	for (int i=0; i<count; ++i) {
		auto& a = node->vehicles.test.list[i];

		// loop over all previous cars (higher prio to yield for)
		for (int j=0; j<i; ++j) {
			auto& b = node->vehicles.test.list[j];
			
			bool dbg = (a.vehicle == sel || a.vehicle == sel2) && (b.vehicle == sel || b.vehicle == sel2);
			yield_for_car(app, node, a, b, dbg);
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

//// Vehicle logic
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

void update_vehicle_suspension (App& app, ActiveVehicle& vehicle, float3 local_accel, float dt) {
	auto& sett = app.network.settings;
	// assume constant mass

	float3 ang = vehicle.suspension_ang;
	float3 vel = vehicle.suspension_ang_vel;

	// spring resitive accel
	//float2 accel = -ang * app.net.settings.suspension_spring_k;
	
	// quadratic for more smooth spring limit (and more wobbly around zero)
	float3 accel = -ang * abs(ang / sett.suspension.max) * sett.suspension.spring_k * 3;
	
	// spring point accel
	accel += local_accel * sett.suspension.accel_fac;
	// spring dampening
	accel -= vel * sett.suspension.spring_damp;

	// apply vel, pos and clamp
	vel += accel * dt;
	vel = clamp(vel, -100, +100);

	ang += vel * dt;
	ang = clamp(ang, -sett.suspension.max, +sett.suspension.max);

	vehicle.suspension_ang = ang;
	vehicle.suspension_ang_vel = vel;
}

void update_vehicle (App& app, Metrics::Var& met, ActiveVehicle* vehicle, float dt) {
	auto& sett = app.network.settings;

	float aggress = vehicle->cit->topspeed_accel_mul();

	assert(vehicle->bez_t < 1.0f);
	float speed_limit = aggress * get_cur_speed_limit(vehicle);

	float old_speed = vehicle->speed;
	float new_speed = old_speed;
	
	vehicle->brake_light = 0.0f;

	// car speed change
	float target_speed = speed_limit * vehicle->brake;
	if (target_speed < 0.33f) target_speed = 0;

	if (target_speed > new_speed) {
		float accel = aggress * calc_car_accel(sett.car_accel, speed_limit, new_speed);
		new_speed += accel * dt;
		new_speed = min(new_speed, target_speed);
	}
	else {
		//new_speed = target_speed; // brake instantly for now
		new_speed -= aggress * calc_car_deccel(sett.car_deccel, speed_limit, new_speed);
		new_speed = max(new_speed, target_speed);

		if (new_speed > 0.33f)
			vehicle->brake_light = 1.0f;
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

		vehicle->state = get_path_state(app.network, vehicle, vehicle->idx, &vehicle->state);

		float blinker = 0;
		if (vehicle->state.state == PathState::SEGMENT || vehicle->state.state == PathState::NODE) {
			if      (vehicle->state.cur_turn == Turns::LEFT ) blinker = -1;
			else if (vehicle->state.cur_turn == Turns::RIGHT) blinker = +1;
		}
		vehicle->blinker = blinker;
	}

	// eval bezier at car front
	auto bez_res = vehicle->state.bezier.eval_with_curv(vehicle->bez_t);
	// remember bezier delta t for next frame
	vehicle->bez_speed = length(bez_res.vel); // bezier t / delta pos
	float3 bez_dir = bez_res.vel / vehicle->bez_speed;

	// actually move car rear using (bogus) trailer formula
	float3 new_front = bez_res.pos;
	
	float3 old_front = vehicle->front_pos;
	float3 old_rear  = vehicle->rear_pos;

	// I don't think this needs to be completely 3d
	auto moveDirs = relative2dir(normalizesafe(old_front - old_rear));
	//float forw_amount = dot(new_front - old_front, forw);

	float car_len = vehicle->car_len();
	float3 ref_point = old_rear + car_len*sett.car_rear_drag_ratio * moveDirs.forw; // Kinda works to avoid goofy car rear movement?

	float3 new_rear = new_front - normalizesafe(new_front - ref_point) * car_len;

	vehicle->front_pos = new_front;
	vehicle->rear_pos  = new_rear;
	
	// totally wack with car_rear_drag_ratio
	vehicle->turn_curv = bez_res.curv; // TODO: to be correct for wheel turning this would need to be computed based on the rear axle

	{
		float3 old_center = (old_front + old_rear) * 0.5f;
		float3 new_center = (new_front + new_rear) * 0.5f;
		float3 center_vel   = dt == 0 ? 0 : (new_center - old_center) / dt;
		float3 center_accel = dt == 0 ? 0 : (center_vel - vehicle->center_vel) / dt;
		
		// accel from world to local space
		//float accel_cap = 30; // we get artefacts with huge accelerations due to discontinuities, cap accel to hide
		//center_accel.y = clamp( dot(center_accel, right), -accel_cap, accel_cap);
		//center_accel.x = clamp( dot(center_accel, forw ), -accel_cap, accel_cap);
		float3 accel_local;
		accel_local.x = dot(center_accel, moveDirs.right);
		accel_local.y = dot(center_accel, moveDirs.forw );
		accel_local.z = dot(center_accel, moveDirs.up   );
		update_vehicle_suspension(app, *vehicle, -accel_local, dt);
		
	#if 0
		if (vehicle->cit == app.interact.selection.get<Person*>()) {
			//printf("%7.3f %7.3f  |  %7.3f %7.3f\n", accel_local.x, accel_local.y, center_vel.x, center_vel.y);
			
			float3 a = moveDirs.right * vehicle->suspension_ang.x
			         + moveDirs.forw  * vehicle->suspension_ang.y
			         + moveDirs.up    * vehicle->suspension_ang.z;
			g_dbgdraw.point(new_center, 0.1f, lrgba(.5f,.5f,.1f,0.5f));
			g_dbgdraw.point(new_center + a*10, 0.1f, lrgba(1,1,0.5f,1));

			g_dbgdraw.arrow(new_front, center_vel, 0.2f, lrgba(0,0,1,1));
			g_dbgdraw.arrow(new_front, center_accel*0.1f, 0.2f, lrgba(0,1,0,1));
			
			float turn_r = 1.0f/vehicle->turn_curv;
			g_dbgdraw.wire_circle(new_rear - moveDirs.right * turn_r, turn_r, lrgba(1,0,0,1), 128);
		}
	#endif

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
	bool force_update_for_dbg = true;
	if (dt > 0.0f || force_update_for_dbg) {
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
	}

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
	debug_person(app, app.interact.selection.get<Person*>(), view);
}

} // namespace network
