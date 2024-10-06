#include "common.hpp"
#include "network.hpp"
#include "app.hpp"

namespace network {

// TODO: remove app as dependency? What is it actually needed for?
	
//// Pathfinding

// Pathfinding ignores lanes other than checking if any lane allows the turn to a node being visited
// Note: lane selection happens later during car path follwing, a few segments into the future
// TODO: rewrite this with segments as the primary item? Make sure to handle start==target
//  and support roads with median, ie no enter or exit buildings with left turn -> which might cause uturns so that segments get visited twice
//  supporting this might require keeping entries for both directions of segments
bool Pathfinding::pathfind (Network& net, Endpoint start, Endpoint target,
		std::vector<Segment*>* result_path) {
	ZoneScoped;
	// use dijkstra

	// TODO: why did I create an optimization and then not use it? it it working? did I want to defer to when I can have more features?
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

	// prepare all nodes
	for (auto& node : net.nodes) {
		node->_cost = INF;
		node->_visited = false;
		node->_q_idx = -1;
		node->_pred = nullptr;
		node->_pred_seg = nullptr;
	}

	// FAILSAFE, TODO: fix!
	// Currently if start == target and forw,backw == true
	//  either pathinding glitches and returns uturn (due to both start nodes being the target nodes)
	//  or (with _pred_seg != target) check, both nodes fail, so rather than fail pathing, just arbitrarily restrict direction
	if (start.seg == target.seg && start.forw && start.backw)
		start.backw = false;
	
	// handle the two start nodes
	// pretend start point is at center of start segment for now
	// forw/backw can restrict the direction allowed for the start segment
	
	if (start.forw) {
		start.seg->node_b->_cost = (start.seg->_length * 0.5f) / start.seg->asset->speed_limit;
		start.seg->node_b->_pred_seg = start.seg;
		unvisited.push({ start.seg->node_b, start.seg->node_b->_cost });
	}
	if (start.backw) {
		start.seg->node_a->_cost = (start.seg->_length * 0.5f) / start.seg->asset->speed_limit;
		start.seg->node_a->_pred_seg = start.seg;
		unvisited.push({ start.seg->node_a, start.seg->node_a->_cost });
	}
	
	net.pathing_count++;

	net._dijk_iter = 0;
	net._dijk_iter_dupl = 0;
	net._dijk_iter_lanes = 0;
	
	while (!unvisited.empty()) {
		net._dijk_iter_dupl++;
		
		// visit node with min cost
		auto _cur_node = unvisited.top();
		Node* cur_node = _cur_node.node;
		unvisited.pop();

		if (cur_node->_visited) continue;
		cur_node->_visited = true;

		net._dijk_iter++;

		// early out optimization
		if (target.seg->node_a->_visited && target.seg->node_b->_visited)
			break; // shortest path found if both target segment nodes are visited

		// Get all allowed turns for incoming segment
		Turns allowed = Turns::NONE;
		for (auto lane : cur_node->_pred_seg->in_lanes(cur_node)) {
			allowed |= lane.get().allowed_turns;
		}

		// update neighbours with new minimum cost
		for (auto& seg : cur_node->segments) {
			for (auto lane : seg->in_lanes(cur_node)) { // This is dumb and makes no sense, TODO: fix it!
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

					unvisited.push({ other_node, other_node->_cost }); // push updated neighbour (duplicate)
				}

				net._dijk_iter_lanes++;
			}
		}
	}
	
	//// make path out of dijkstra graph
	
	// additional distances from a and b of the target segment
	float dist_from_a = 0.5f;
	float dist_from_b = 0.5f;

	Node* end_node = nullptr;
	float a_cost = target.seg->node_a->_cost + dist_from_a / target.seg->asset->speed_limit;
	float b_cost = target.seg->node_b->_cost + dist_from_b / target.seg->asset->speed_limit;

	// do not count final node if coming from target segment, to correctly handle start == target
	if (target.seg->node_a->_pred_seg && target.seg->node_a->_pred_seg != target.seg) {
		end_node = target.seg->node_a;
	}
	if (target.seg->node_b->_pred_seg && target.seg->node_b->_pred_seg != target.seg) {
		// if both nodes count, choose end node that end up fastest
		if (!end_node || b_cost < a_cost) {
			end_node = target.seg->node_b;
		}
	}

	if (!end_node)
		return false; // no path found
		
	assert(end_node->_cost < INF);

	std::vector<Segment*> reverse_segments;
	reverse_segments.push_back(target.seg);

	Node* cur = end_node;
	while (cur) {
		assert(cur->_pred_seg);
		reverse_segments.push_back(cur->_pred_seg);
		cur = cur->_pred;
	}
	assert(reverse_segments.size() >= 2); // code currently can't handle single segment path
	// TODO: make that possible and then handle make driving into building on other side of road possible? Or should we just have it drive around the block for this?

	for (int i=(int)reverse_segments.size()-1; i>=0; --i) {
		result_path->push_back(reverse_segments[i]);
	}

	return true;
}
void debug_last_pathfind (Network& net, View3D& view) {
	
	static bool visualize = false;
	if (imgui_Header("debug_last_pathfind", true)) {
		ImGui::Checkbox("visualize", &visualize);
		ImGui::PopID();
	}
	
	if (!visualize) return;
	
	float max_cost = 0;
	for (auto& node : net.nodes) {
		if (node->_visited) {
			max_cost = max(max_cost, node->_cost);
		}
	}

	for (auto& node : net.nodes) {
		if (node->_visited) {
			float cost_a = node->_cost / max_cost;
			lrgba col = lerp(lrgba(1,0,1,1), lrgba(1,0,0,1), clamp(cost_a, 0.0f, 1.0f));

			g_dbgdraw.wire_circle(node->pos, node->_radius, col);

			g_dbgdraw.text.draw_text(prints("%.0f", node->_cost), 30,
				1, g_dbgdraw.text.map_text(node->pos, view));

			if (node->_pred_seg) {
				float3 pos = (node->_pred_seg->pos_a + node->_pred_seg->pos_b) * 0.5f;

				g_dbgdraw.arrow(view, pos, node->pos - pos, 5, lrgba(0,1,1,1));
			}
		}
	}
}

////

bool VehNav::pathfind (Network& net, VehNavPoint const& start, VehNavPoint const& target) {
	assert(path.empty());
	
	if (!Pathfinding::pathfind(net, { start.seg }, { target.seg }, &path))
		return false;
	
	this->start = start;
	this->target = target;
	
	s = _step(net, 0, nullptr);
	return true;
}
bool VehNav::repath (Network& net, VehNavPoint const& new_target) {
	if (s.motion == END)
		return false;
	
	int num_seg = (int)path.size();
	int num_moves = num_seg + (num_seg-1) + 2;

	int i = (s.idx-1)/2;
	assert(s.idx < num_moves);
	assert(i < num_seg);

	// Keep start navpoint, change target
	VehNav new_nav;
	new_nav.start = start;
	new_nav.target = new_target;
	
	Pathfinding::Endpoint path_start;
	State dummy_state;
	State* pdummy_state = nullptr;
	int new_idx;

	if (s.motion == START) {
		assert(s.next_lane);
		assert(num_seg >= 2);

		auto* node = Node::between(path[0], path[1]);
		LaneDir dir = path[0]->get_dir_to_node(node);

		// repath while staying in start bezier, make sure first lane goes in same direction
		// TODO: this is not always correct, might still teleport into different lane!
		path_start.seg = start.seg;
		path_start.forw  = dir == LaneDir::FORWARD;
		path_start.backw = dir != LaneDir::FORWARD;

		new_idx = 0; // Start at START
	}
	else {
		// there is no next segment because we are on the target segment
		// This case should be safe, if handles specially, but just disallow repathing
		if (i+1 >= num_seg)
			return false;
		
		assert(i+1 < num_seg);
		assert(s.cur_lane);
		
		auto* node = Node::between(path[i], path[i+1]);
		LaneDir dir = path[i+1]->get_dir_from_node(node);

		// repath while keeping cur and next lane (keep same path through node)
		// TODO: this is not always correct, might still teleport into different lane!
		new_nav.path.push_back(path[i]);
		path_start.seg = path[i+1];
		path_start.forw  = dir == LaneDir::FORWARD;
		path_start.backw = dir != LaneDir::FORWARD;
		
		if (s.motion == SEGMENT) {
			dummy_state.next_lane = s.cur_lane; // sets cur_lane currectly

			new_idx = 1; // Start at SEGMENT
		}
		else {
			assert(s.motion == NODE);
			assert(s.next_lane);
			
			dummy_state.cur_lane  = s.cur_lane; // sets cur_lane currectly
			dummy_state.next_lane = s.next_lane; // sets next_lane currectly

			new_idx = 2; // Start at NODE
		}
		pdummy_state = &dummy_state;
	}

	if (!Pathfinding::pathfind(net, path_start, { new_target.seg }, &new_nav.path))
		return false;

	new_nav.s = new_nav._step(net, new_idx, &dummy_state);

	std::swap(*this, new_nav);
	return true;
}

// Return first lane connector for next segment in path if available
SegLane pick_stay_in_lane (SegLane const& cur_lane, Segment const* next_seg) {
	assert(cur_lane);
	for (auto& conn : cur_lane.get().connections) {
		if (conn.seg == next_seg) {
			return conn;
		}
	}
	return SegLane{};
}
// Pick random next lane that has correct turn arrow for segment later in the path
SegLane pick_random_lane (Random& rand, Segment::LanesRange& avail_lanes, Segment* target_seg,
		SegLane default_lane, bool exclude_default=false) {
#if 0
	auto choose = [&] (SegLane lane) {
		return any_set(lane.get().allowed_turns, req_turn) &&
			(exclude_default ? lane != default_lane : true);
	};

	int count = 0;
	for (auto lane : next_lanes) {
		if (choose(lane))
			count++;
	}
		
	if (count > 0) {
		int choice = rand.uniformi(0, count);
		int idx = 0;
		for (auto lane : next_lanes) {
			if (choose(lane) && idx++ == choice)
				return lane;
		}
	}
	// if no lanes were chosen for random pick either because exclude_default or missing allowed turn (pathfinding bug?)
	return default_lane;
#else
	std::vector<SegLane> choices;
	choices.reserve(8);

	for (auto lane : avail_lanes) {
		for (auto& conn : lane.get().connections) {
			if (conn.seg == target_seg && (exclude_default ? lane != default_lane : true)) {
				choices.push_back(lane);
				break; // only push each lane once
			}
		}
	}
		
	if (choices.size() > 0) {
		int choice = rand.uniformi(0, (int)choices.size());
		return choices[choice];
	}
	// if no lanes were chosen for random pick either because exclude_default or missing allowed turn (pathfinding bug?)
	return default_lane;
#endif
};

SegLane VehNav::pick_lane (Network& net, Random& rand, int seg_i, SegLane prev_lane) const {
	int num_seg = (int)path.size();
	assert(seg_i >= 0 && seg_i < num_seg);

	// forward iteration, try to follow lane until we have to switch lanes to make turn (or end of path is reached)
	int end_seg = -1;

	std::vector<SegLane> stay_lanes;
	std::vector<SegLane> prediced_lanes;

	for (int i=seg_i; i<num_seg; ++i) {
		end_seg = i;

		auto* cur_seg  = path[i];
		auto* next_seg = i+1 < num_seg ? path[i+1] : nullptr;
			
		// Find avail lanes for current path segment
		Segment::LanesRange cur_lanes;
		if (next_seg) {
			auto* node = Node::between(cur_seg, next_seg);
			cur_lanes = cur_seg->in_lanes(node);
		}
		else {
			SegLane lane = SegLane{ cur_seg, 0 };
			// no next lane, end of path default to outer
			if (prev_lane) {
				auto* node = Node::between(prev_lane.seg, cur_seg);
				lane = cur_seg->out_lanes(node).outer();
			}
			else {
				assert(false); // uhhh
			}
				
			stay_lanes.push_back(lane);
			break;
		}
			
		// Pick cur lane based on previous lane
		SegLane lane;
		if (prev_lane) {
			// try to stay on previous lane
			lane = pick_stay_in_lane(prev_lane, cur_seg);
		}
		else {
			// no previous lane, default to outer
			lane = cur_lanes.outer();
		}
			
		if (lane) {
			// check if lane also lets us stay in lane for next seg
			SegLane lane1 = pick_stay_in_lane(lane, next_seg);
			if (lane1) {
				prev_lane = lane;
				stay_lanes.push_back(lane);
				continue;
			}
		}
			
		// couldn't stay in lane from prev -> next, pick any available
		lane = pick_random_lane(rand, cur_lanes, next_seg, cur_lanes.outer());
		stay_lanes.push_back(lane);
		break;
	}

	// backwards iteration, try to follow lanes backwards from lane we had to switch to
	auto follow_connection_backwards = [] (Segment* prev, SegLane& cur) {
		auto* node = Node::between(prev, cur.seg);
		auto lanes = prev->in_lanes(node);
			
		SegLane best_lane = SegLane{};
		int best_diff = INT_MAX;

		// try to find prev lane that has connection to cur lane
		for (auto lane : lanes) {
			for (auto& conn : lane.get().connections) {
				// TODO: could be handled via best_lane as well (diff=0)
				if (conn == cur)
					return lane; // first lane that connects to cur lane found

				if (conn.seg == cur.seg) {
					// lane does not connect to cur lane, but to right segment
					int diff = abs(conn.lane - cur.lane);
					if (diff < best_diff) {
						best_lane = lane;
						best_diff = diff;
					}
				}
			}
		}

		// else use closest connection if found
		return best_lane;
	};

	prediced_lanes.resize(stay_lanes.size());
	prediced_lanes.back() = stay_lanes.back();

	for (int i=end_seg; i>seg_i; --i) {
		auto* prev_seg = path[i-1];
		auto cur_lane = prediced_lanes[i -seg_i];
			
		auto lane = follow_connection_backwards(prev_seg, cur_lane);
		if (lane) {
			prediced_lanes[i-1 -seg_i] = lane;
		}
		else {
			break; // no backwards connection found, will stay in current lane and switch later (?)
		}
	}
		
	//
	auto stay_lane     = stay_lanes.front();
	auto prediced_lane = prediced_lanes.front();

	assert(stay_lane && stay_lane.seg == path[seg_i]);
	if (prediced_lane)
		assert(prediced_lane.seg == path[seg_i]);

	auto lane = stay_lane;
	if (prediced_lane && rand.chance(net._lane_switch_chance)) {
		lane = prediced_lane;
	}
	return lane;
};

Bezier3 start_bezier (SegLane const& lane, VehNavPoint const& start, float* out_t) {
	auto lane_bez = lane._bezier();
	float len = lane_bez.approx_len(4);
	float t = min(0.5f + 5 / len, 1.0f);
		
	float3 point  = start.pos;
	float3 ctrl   = lane_bez.eval(0.5f).pos;
	float3 on_seg = lane_bez.eval(t).pos;

	*out_t = t;
	return Bezier3(point, ctrl, ctrl, on_seg);
}
Bezier3 end_bezier (SegLane const& lane, VehNavPoint const& end, float* out_t) {
	auto lane_bez = lane._bezier();
	float len = lane_bez.approx_len(4);
	float t = max(0.5f - 5 / len, 0.0f); // curve such that the endpoint is 5m earlier on the lane than the middle point

	float3 point  = end.pos;
	float3 ctrl   = lane_bez.eval(0.5f).pos;
	float3 on_seg = lane_bez.eval(t).pos;

	*out_t = t;
	return Bezier3(on_seg, ctrl, ctrl, point);
}

float get_speed_limit (VehNav::MotionType motion, SegLane cur_lane={}, SegLane next_lane={}) {
	switch (motion) {
		case VehNav::SEGMENT: {
			assert(cur_lane);
			return cur_lane.seg->asset->speed_limit;
		}
		case VehNav::NODE: {
			assert(cur_lane && next_lane);
			float a = cur_lane .seg->asset->speed_limit;
			float b = next_lane.seg->asset->speed_limit;
			return min(a, b); // TODO: ??
		}
		default: {
			return 20 / KPH_PER_MS;
		}
	}
}
float get_curvature_speed_limit (float curv) {
	float max_accel = 6.0f;
	// centripedal accel formula: a = v^2 / r <=> a = v^2 * curv
	float max_speed = sqrt(max_accel / (curv + 0.001f));
	return max(max_speed, 5.0f / KPH_PER_MS);
}
// Ideally apply to all beziers, not just curves
float get_curve_speed_limit (Bezier3 const& bez, SegLane cur_lane, SegLane next_lane) {
	float seg_limit = get_speed_limit(VehNav::MotionType::NODE, cur_lane, next_lane);

	// sample multiple times to slow down in S-curves as well
	float curv =     abs(bez.eval_with_curv(0.25f).curv);
	curv = max(curv, abs(bez.eval_with_curv(0.5f ).curv));
	curv = max(curv, abs(bez.eval_with_curv(0.75f).curv));

	float curv_limit = get_curvature_speed_limit(curv);

	return min(seg_limit, curv_limit);
}

// TODO: see if it might not be better to write state-machine like query function that only ever returns current state
//  then use call it with +1 to get any info needed for next lanes etc.
VehNav::State VehNav::_step (Network& net, int idx, State* prev_state) const {
	State s = {};
	s.idx = idx;

	int num_seg = (int)path.size();
	int num_moves = num_seg + (num_seg-1) + 2;
	assert(num_seg >= 1);

	// Make lane selection deterministic for path visualization
	// TODO: this requirement might go away once I do lane selections more robustly
	//   Might still want to keep determinism based on car id + path progress int, just so reloading a save state results in the same behavior?
	auto seeded_rand = Random(hash(idx, (uint64_t)this));
	
	if (idx == 0) {
		s.motion = MotionType::START;
		
		//s.cur_lane = {};
		s.next_lane = pick_lane(net, seeded_rand, 0, SegLane{});

		s.bezier = start_bezier(s.next_lane, start, &s.next_start_t);

		s.cur_speedlim  = get_speed_limit(MotionType::START);
		s.next_speedlim = get_speed_limit(MotionType::SEGMENT, s.next_lane);
	}
	else if (idx == num_moves-1) {
		s.motion = MotionType::END;

		auto prev_lane = prev_state->cur_lane;
		//s.cur_lane = {};
		//s.next_lane = {};

		float t;
		s.bezier = end_bezier(prev_lane, target, &t);

		s.cur_speedlim  = get_speed_limit(MotionType::END);
		s.next_speedlim = 1; // stop at end of last bezier, TODO: how to customize based on target? ex. for underground parking entrance
	}
	else {
		int i = (s.idx-1)/2;
		assert(i >= 0 && i < num_seg);
		
		if ((idx-1) % 2 == 0) {
			s.motion = MotionType::SEGMENT;

			// prev call already determinted next_lane
			s.cur_lane = prev_state->next_lane;

			if (idx+1 == num_moves-1) {
				// already on target lane, need end_t
				end_bezier(s.cur_lane, target, &s.end_t);
				//s.next_lane = {};
			}
			else {
				s.next_lane = pick_lane(net, seeded_rand, i+1, s.cur_lane);
			}

			s.cur_vehicles  = &s.cur_lane.vehicles().list;
			
			s.bezier = s.cur_lane._bezier();

			s.cur_speedlim = get_speed_limit(MotionType::SEGMENT, s.cur_lane);
			// TODO: can this be written more concisely?
			if (s.next_lane) {
				Node* cur_node = Node::between(s.cur_lane.seg, s.next_lane.seg);
				auto curve_bez = cur_node->calc_curve(s.cur_lane, s.next_lane);
				s.next_speedlim = get_curve_speed_limit(curve_bez, s.cur_lane, s.next_lane);
			}
			else {
				s.next_speedlim = get_speed_limit(MotionType::END);
			}
		}
		else {
			s.motion = MotionType::NODE;
			
			// lanes were already determined in prev call
			s.cur_lane = prev_state->cur_lane;
			s.next_lane = prev_state->next_lane;

			assert(s.cur_lane && s.next_lane);
			Node* cur_node = s.next_lane ? Node::between(s.cur_lane.seg, s.next_lane.seg) : nullptr;
			assert(cur_node);
			
			//s.cur_vehicles  = nullptr;
			
			s.bezier = cur_node->calc_curve(s.cur_lane, s.next_lane);

			s.cur_speedlim  = get_curve_speed_limit(s.bezier, s.cur_lane, s.next_lane);
			s.next_speedlim = get_speed_limit(MotionType::SEGMENT, s.next_lane);
		}
	}

	return s;
}

void VehNav::visualize (OverlayDraw& overlay, Network& net, SimVehicle* vehicle, bool skip_next_node, lrgba col) {
	State copy = s;
	float start_t = vehicle->bez_t;
	
	// corretly handle first motion
	while (!(copy.motion == END && start_t >= copy.end_t)) {

		if (skip_next_node && copy.motion == NODE) {
			skip_next_node = false;
		}
		else {
			overlay.curves.push_arrow(copy.bezier,
				float2(2, 1), OverlayDraw::TEXTURE_THIN_ARROW, col, float2(start_t, copy.end_t));
		}
		
		start_t = copy.next_start_t;
		if (copy.motion == END) break; // avoid step after last motion
		
		// call step for visualize, this might be a bad idea if it modifies path (chosen lanes)
		copy = _step(net, copy.idx + 1, &copy);
	}
}

////
float _brake_for_dist (float obstacle_dist) {
	return clamp(map(obstacle_dist, 0.0f, 8.0f), 0.0f, 1.0f);
}
void brake_for_dist (SimVehicle* vehicle, float obstacle_dist) {
	float brake = _brake_for_dist(obstacle_dist);
	vehicle->brake = min(vehicle->brake, brake);
}

//// Visualization
void overlay_lane_vehicle (App& app, SimVehicle* vehicle, lrgba col, int tex) {
	// TODO: simple extrapolation for now, abstract this away and roll this into the logic used by node traffic sim
	// to have one place where covered bezier ranges are determined
	// (enable asking for car front and back bezier t, coresponding beziers (and ones inbetween if car longer than 1 segment/node etc.)
	// sample bezier with t to determine worldspace pos, or draw bezier with overlay etc.
	
	Bezier3 cur_bez = vehicle->nav.get_state().bezier;
	float t1 = vehicle->bez_t;
	float t0 = vehicle->bez_t - vehicle->vehicle_asset->car_len() / vehicle->bez_speed; // t - len / (dlen / dt) => t - dt
	if (t0 < 0) {
		// car rear on different bezier!
		t0 = 0;

		// TODO: actually enable computing previous bezier!
	}
	
	app.overlay.curves.push_bezier(cur_bez, float2(LANE_COLLISION_R*2, 1), tex, col, float2(t0,t1));
}

void dbg_node_lane_alloc (App& app, Node* node) {
	
	auto dbg_avail_space = [&] (SegLane& lane_out, SimVehicle* a) {
		// interpolate lane bezier instead of trying to go call overlay_lane_vehicle on vehicle
		// because vehicle is not actually in this spot yet! remember we allocate lane space by placing virtual cars!
		
		auto bez = lane_out._bezier();
		float t1 = lane_out.vehicles().avail_space / lane_out.seg->_length;

		float bez_speed = length(bez.eval(t1).vel);
		float t0 = t1 - a->vehicle_asset->car_len() / bez_speed; // TODO: cache accurate k in vehicle and eliminate this calculation!
		
		app.overlay.curves.push_bezier(bez, float2(LANE_COLLISION_R*2, 1), OverlayDraw::PATTERN_STRIPED, lrgba(a->tint_col, 0.8f), float2(t0,t1));
	};

	for (auto& seg : node->segments) {
		for (auto lane_out : seg->out_lanes(node)) {
			auto& avail_space = lane_out.vehicles().avail_space;

			avail_space = lane_out.seg->_length;
			for (auto* a : lane_out.seg->vehicles.lanes[lane_out.lane].list.list) {
				dbg_avail_space(lane_out, a);
				avail_space -= a->vehicle_asset->car_len() + SAFETY_DIST;
			}
		}
	}
	
	// allocate space in priority order and remember blocked cars
	for (auto& v : node->vehicles.test.list) {
		auto& state = v.vehicle->nav.get_state();
		if (state.get_cur_node() != node) {
			// already in outgoing lane
			continue;
		}

		auto& avail_space = v.conn.conn.b.vehicles().avail_space;
		if (avail_space >= v.vehicle->vehicle_asset->car_len()) {
			dbg_avail_space(v.conn.conn.b, v.vehicle);
			avail_space -= v.vehicle->vehicle_asset->car_len() + SAFETY_DIST;
		}
	}
}
bool dbg_conflicts (App& app, Node* node, SimVehicle* vehicle);

void debug_node (App& app, Node* node, View3D const& view) {
	if (!node) return;
	
	static bool debug_priority_order = false;
	static bool debug_node_lane_alloc = false;
	static bool show_lane_connections = false;
	static bool debug_vehicle_lists = false;
	if (imgui_Header("debug_node", true)) {
		ImGui::Checkbox("debug_priority_order", &debug_priority_order);
		ImGui::Checkbox("debug_node_lane_alloc", &debug_node_lane_alloc);
		ImGui::Checkbox("show_lane_connections", &show_lane_connections);
		ImGui::Checkbox("debug_vehicle_lists", &debug_vehicle_lists);

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
					app.overlay.curves.push_arrow(bez, float2(3.5f, 1), OverlayDraw::TEXTURE_THIN_ARROW, col);
				}
			}
		}
	}

	if (debug_vehicle_lists) {
		auto show_vehicle = [&] (SimVehicle* v) {
			ImGui::TextColored(lrgba(v->tint_col,1), "%s", v->vehicle_asset->name.c_str());
		};

		if (ImGui::TreeNodeEx("NodeVehicles", ImGuiTreeNodeFlags_DefaultOpen)) {
			for (auto& v : node->vehicles.test.list) {
				show_vehicle(v.vehicle);
			}
			ImGui::TreePop();
		}
		for (auto* seg : node->segments) {
			for (auto lane : seg->in_lanes(node)) {
				if (ImGui::TreeNodeEx("In  LaneVehicles", ImGuiTreeNodeFlags_DefaultOpen)) {
					for (auto* v : lane.vehicles().list.list) {
						show_vehicle(v);
					}
					ImGui::TreePop();
				}
			}
		
			for (auto lane : seg->out_lanes(node)) {
				if (ImGui::TreeNodeEx("Out LaneVehicles", ImGuiTreeNodeFlags_DefaultOpen)) {
					for (auto* v : lane.vehicles().list.list) {
						show_vehicle(v);
					}
					ImGui::TreePop();
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
void debug_vehicle (App& app, Vehicle& veh, SimVehicle& sim_veh, View3D const& view) {
	static bool visualize_lane_section = false;
	static bool visualize_nav = true;
	static bool visualize_conflicts = true;
	static bool visualize_bones = false;
	static ValuePlotter speed_plot = ValuePlotter();
	speed_plot.push_value(sim_veh.speed);

	if (imgui_Header("debug_person", true)) {
		ImGui::Checkbox("visualize_lane_section", &visualize_lane_section);
		ImGui::Checkbox("visualize_nav", &visualize_nav);
		ImGui::Checkbox("visualize_conflicts", &visualize_conflicts);
		ImGui::Checkbox("visualize_bones", &visualize_bones);
		
		ImGui::Separator();
		ImGui::TextColored(lrgba(veh.col, 1), "debug person");
	
		ImGui::Text("Speed Limit: %7s", app.options.format_speed(sim_veh.nav.get_state().cur_speedlim).c_str());
		ImGui::Text("Speed: %7s",       app.options.format_speed(sim_veh.speed).c_str());

		speed_plot.imgui_display("speed", 0.0f, 100/KPH_PER_MS);

		ImGui::PopID();
	}
	
	if (visualize_lane_section)
		overlay_lane_vehicle(app, &sim_veh, lrgba(veh.col, 1), OverlayDraw::PATTERN_SOLID);
	
	bool did_viz_conflicts = false;

	auto* cur_node = sim_veh.nav.get_state().get_cur_node();
	if (visualize_conflicts && cur_node) {
		did_viz_conflicts = dbg_conflicts(app, cur_node, &sim_veh);
	}

	if (visualize_nav) {
		sim_veh.nav.visualize(app.overlay, app.network, &sim_veh, did_viz_conflicts);
	}

	if (visualize_bones) {
		for (auto& bone : veh.asset->bone_mats) {
			auto pos = sim_veh.calc_pos();

			auto mat = translate(pos.pos) * rotate3_Z(pos.ang) * bone.bone2mesh;

			auto p = (float3)(mat * float4(0,0,0,1));
			//g_dbgdraw.point(pos, 0.01f, lrgba(0,0,0,1));
			g_dbgdraw.line(p, (float3)(mat * float4(.25f,0,0,1)), lrgba(1,0,0,1));
			g_dbgdraw.line(p, (float3)(mat * float4(0,.25f,0,1)), lrgba(0,1,0,1));
			g_dbgdraw.line(p, (float3)(mat * float4(0,0,.25f,1)), lrgba(0,0,1,1));
		}
	}
}

#if 1
void dbg_brake_for (App& app, SimVehicle* cur, float dist, float3 obstacle, lrgba col) {
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
void dbg_brake_for (App& app, SimVehicle* cur, float dist, float3 obstacle, lrgba col) {}
#endif
auto _SimVehicle_sel = [] (sel_ptr& sel) -> SimVehicle* {
	auto* pers = sel.get<Person*>();
	if (!pers) return nullptr;
	auto* trip = pers->owned_vehicle->get_trip();
	return trip ? &trip->sim : nullptr;
};

void _FORCEINLINE dbg_brake_for_vehicle (App& app, SimVehicle* cur, float dist, SimVehicle* obstacle) {
	if (_SimVehicle_sel(app.interact.selection) == cur) {
		float3 center = (obstacle->rear_pos + obstacle->front_pos) * 0.5f;
		dbg_brake_for(app, cur, dist, center, lrgba(1,0.1f,0,1));
	}
}
void _FORCEINLINE dbg_brake_for_blocked_lane_start (App& app, NodeVehicle& v, float dist, SegLane& lane) {
	if (_SimVehicle_sel(app.interact.selection) == v.vehicle) {
		dbg_brake_for(app, v.vehicle, dist, lane._bezier().a, lrgba(0.2f,0.8f,1,1));
	}
}
void _FORCEINLINE dbg_brake_for_blocked_lane_end (App& app, NodeVehicle& v, float dist, SegLane& lane) {
	if (_SimVehicle_sel(app.interact.selection) == v.vehicle) {
		dbg_brake_for(app, v.vehicle, dist, lane._bezier().b, lrgba(0.2f,0.8f,1,1));
	}
}

//// Segment logic
void update_segment (App& app, Segment* seg) {
	for (auto& lane : seg->vehicles.lanes) {
		// brake for car in front
		for (int i=1; i<(int)lane.list.list.size(); ++i) {
			SimVehicle* prev = lane.list.list[i-1];
			SimVehicle* cur  = lane.list.list[i];
			
			// approx seperation using cur car bez_speed
			float dist = (prev->bez_t - cur->bez_t) * cur->bez_speed - (prev->vehicle_asset->car_len() + 1);

			brake_for_dist(cur, dist);
			dbg_brake_for_vehicle(app, cur, dist, prev);
		}
	}
}

//// Vehicle logic
Conflict check_conflict (CachedConnection const& a, CachedConnection const& b) {
	assert(a.conn != b.conn);
	
	// These are only needed for dbg vis, and on conflict cache miss
	// since computing conflicts is already expensive N(COLLISION_STEPS^2 * 4), we could just compute the points for the 2 beziers on the fly
	// which is N(2 * (COLLISION_STEPS+1)) for the two beziers (Note: L and R can be computed at the same time)
	float2 a_pointsL[COLLISION_STEPS+1];
	float2 a_pointsR[COLLISION_STEPS+1];
	float2 b_pointsL[COLLISION_STEPS+1];
	float2 b_pointsR[COLLISION_STEPS+1];
	{
		auto bez2d = (Bezier2)a.bezier;
		bez2d.calc_points(a_pointsL, COLLISION_STEPS+1, -LANE_COLLISION_R);
		bez2d.calc_points(a_pointsR, COLLISION_STEPS+1, +LANE_COLLISION_R);
	}
	{
		auto bez2d = (Bezier2)b.bezier;
		bez2d.calc_points(b_pointsL, COLLISION_STEPS+1, -LANE_COLLISION_R);
		bez2d.calc_points(b_pointsR, COLLISION_STEPS+1, +LANE_COLLISION_R);
	}

	float u0 = INF;
	float v0 = INF;
	float u1 = -INF;
	float v1 = -INF;

	for (int i=0; i<COLLISION_STEPS; ++i) {
		float2 aL_dir = a_pointsL[i+1] - a_pointsL[i];
		float2 aR_dir = a_pointsR[i+1] - a_pointsR[i];

		for (int j=0; j<COLLISION_STEPS; ++j) {
			float2 bL_dir = b_pointsL[j+1] - b_pointsL[j];
			float2 bR_dir = b_pointsR[j+1] - b_pointsR[j];
			
			float line_u, line_v;

			if (line_line_seg_intersect(a_pointsL[i], aL_dir, b_pointsL[j], bL_dir, &line_u, &line_v)) {
				float u = line_u + (float)i;
				float v = line_v + (float)j;

				u0 = min(u0, u);
				v0 = min(v0, v);
				u1 = max(u1, u);
				v1 = max(v1, v);
			}

			if (line_line_seg_intersect(a_pointsR[i], aR_dir, b_pointsL[j], bL_dir, &line_u, &line_v)) {
				float u = line_u + (float)i;
				float v = line_v + (float)j;

				u0 = min(u0, u);
				v0 = min(v0, v);
				u1 = max(u1, u);
				v1 = max(v1, v);
			}

			if (line_line_seg_intersect(a_pointsL[i], aL_dir, b_pointsR[j], bR_dir, &line_u, &line_v)) {
				float u = line_u + (float)i;
				float v = line_v + (float)j;

				u0 = min(u0, u);
				v0 = min(v0, v);
				u1 = max(u1, u);
				v1 = max(v1, v);
			}

			if (line_line_seg_intersect(a_pointsR[i], aR_dir, b_pointsR[j], bR_dir, &line_u, &line_v)) {
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

bool dbg_conflicts (App& app, Node* node, SimVehicle* vehicle) {
	int idx = kiss::indexof(node->vehicles.test.list, vehicle, [&] (NodeVehicle const& l, SimVehicle* r) { return l.vehicle == r; });
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
	float bez_t = a.front_k < 0 ? 0.0f : clamp(a.vehicle->bez_t, 0.0f, 1.0f);
	app.overlay.curves.push_arrow(a.conn.bezier, sz, OverlayDraw::TEXTURE_THICK_ARROW, lrgba(0.9f,0.9f,0.1f, 0.9f), float2(bez_t, 1));
	
	// loop over all previous cars (higher prio to yield for)
	for (int j=0; j<idx; ++j) {
		auto& b = node->vehicles.test.list[j];

		Conflict conf;
		if (!has_conflict(node, a, b, conf)) continue;
		
		// conflict on visualized vehicle's path as red striped zone
		app.overlay.curves.push_bezier(a.conn.bezier, sz, OverlayDraw::PATTERN_STRIPED, lrgba(1.0f,0.02f,0.02f, 1), float2(conf.a_t0, conf.a_t1));
	}

	// draw red arrows last to overlap all striped regions (looks nicer)
	for (int j=0; j<idx; ++j) {
		auto& b = node->vehicles.test.list[j];

		Conflict conf;
		if (!has_conflict(node, a, b, conf)) continue;
		
		// yielded-to vehicle's path through node as thin red arrow
		app.overlay.curves.push_arrow(b.conn.bezier, sz, OverlayDraw::TEXTURE_THIN_ARROW, lrgba(1.0f,0.08f,0.08f, 0.9f));
	}

	return true;
}

/*
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
}*/

NodeVehicle* get_left_vehicle (Node* node, NodeVehicle& a, NodeVehicle& b) {
	auto b_to_a = classify_turn(node, a.conn.conn.a.seg, b.conn.conn.a.seg);
	auto a_turn = classify_turn(node, a.conn.conn.a.seg, a.conn.conn.b.seg);
	auto b_turn = classify_turn(node, b.conn.conn.a.seg, b.conn.conn.b.seg);
	if (b_to_a == Turns::RIGHT) return &a; // a is left if incoming segment of b is right of a
	if (b_to_a == Turns::LEFT ) return &b;
	if (a_turn == Turns::LEFT && b_turn != Turns::LEFT) return &a; // a is left if it does a left turn
	if (b_turn == Turns::LEFT && a_turn != Turns::LEFT) return &b;
	return nullptr;
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

	auto get_incoming_lane = [node] (NodeVehicle& v) {
		if (v.vehicle->nav.get_state().get_cur_node() == node && v.vehicle->nav.get_state().motion == VehNav::SEGMENT)
			return v.vehicle->nav.get_state().cur_lane;
		return SegLane{};
	};

	auto a_lane = get_incoming_lane(a);
	auto b_lane = get_incoming_lane(b);
	if (a_lane && a_lane == b_lane) {
		// in same lane, failsafe for bug
		assert(a_lane.vehicles().list.list == b_lane.vehicles().list.list);
		int a_idx = indexof(a_lane.vehicles().list.list, a.vehicle);
		int b_idx = indexof(a_lane.vehicles().list.list, b.vehicle);
		assert(a_idx >= 0 && b_idx >= 0 && a_idx != b_idx);
		return a_idx > b_idx;
	}

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
			left_vehicle = get_left_vehicle(node, a, b);
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

void update_node (App& app, Node* node, float dt) {
	bool node_dbg = app.interact.selection.get<Node*>() == node;

	auto* sel  = _SimVehicle_sel(app.interact.selection);
	auto* sel2 = _SimVehicle_sel(app.interact.hover);

	// update traffic light
	if (node->traffic_light) {
		assert(node->traffic_light);
		node->traffic_light->update(node, dt);
	}
	
	// compute avail space in outgoing lanes taking into acount vehicles already in it
	for (auto& seg : node->segments) {
		for (auto lane_out : seg->out_lanes(node)) {
			auto& avail_space = lane_out.vehicles().avail_space;

			avail_space = lane_out.seg->_length;
			for (auto* a : lane_out.seg->vehicles.lanes[lane_out.lane].list.list) {
				avail_space -= a->vehicle_asset->car_len() + SAFETY_DIST;
			}
		}
	}
	
	auto track_node_vehicle = [node] (SimVehicle* v, VehNav::State const& state) {
		NodeVehicle nv;
		nv.vehicle = v;
		nv.wait_time = 0;
		nv.conn.conn = { state.cur_lane, state.next_lane };
		nv.conn.bezier = node->calc_curve(nv.conn.conn.a, nv.conn.conn.b);
		nv.conn.bez_len = nv.conn.bezier.approx_len(COLLISION_STEPS);

		return nv;
	};

	// Add vehicles close to intersection to tracked list
	for (auto& seg : node->segments) {
		for (auto lane : seg->in_lanes(node)) {
			auto it = lane.vehicles().list.list.begin();
			// first part of list, add vehicles if not already in list
			while (it != lane.vehicles().list.list.end()) { auto* v = *it++;
				if (node->vehicles.test.contains(v)) continue; // TODO: Expensive contains with vector

				float dist = (1.0f - v->bez_t) * v->bez_speed;
				if (dist < 10.0f || v == lane.vehicles().list.list.front()) {
					auto& state = v->nav.get_state();
					auto* n = state.get_cur_node();
					if (n == node) {
						node->vehicles.test.add(track_node_vehicle(v, state));
					}
				}
				else {
					// not in list and should not be in list
					break;
				}
			}

			// FAILSAFE: deadlock if vehicles get added to node according to distance threshold, but order in lane is mismatched due to whatever reason
			// remaing part of list, remove vehicles if in list to avoid bug
			while (it != lane.vehicles().list.list.end()) { auto* v = *it++;
				node->vehicles.test.try_remove(v);
			}
		}
	}
	
	auto update_ks = [&] (NodeVehicle& v, VehNav::State const& state) {
		if (state.get_cur_node() == node) {
			// ingoing lane
			if (state.motion == VehNav::SEGMENT) {
				// extrapolate and map from negative to 0
				v.front_k = (v.vehicle->bez_t - 1.0f) * v.vehicle->bez_speed;
			}
			// on node
			else {
				assert(state.motion == VehNav::NODE);
				// approximate by just mapping t (which is wrong)
				v.front_k = v.vehicle->bez_t * v.conn.bez_len;
			}
		}
		// assume outgoing lane (update should never move vehicle by more than one segment per tick!)
		else {
			// extrapolate and map from negative to 0
			v.front_k = v.vehicle->bez_t * v.vehicle->bez_speed + v.conn.bez_len;
		}
		
		v.rear_k = v.front_k - v.vehicle->vehicle_asset->car_len();
	};

	// update each tracked vehicle
	// allocate space in priority order and remember blocked cars
	for (auto& v : node->vehicles.test.list) {
		auto& state = v.vehicle->nav.get_state();

		// compute intersection progress value 'k'
		update_ks(v, state);
		
		v.blocked = false;
		v.wait_time += dt; // increment wait time

		if (state.get_cur_node() != node) {
			// already in outgoing lane (don't need to wait and avoid counting avail space twice)
			continue;
		}

		bool incoming_lane = state.motion == VehNav::SEGMENT;
		if (incoming_lane && node->traffic_light) {
			assert(v.front_k < 0.001f);
			// still in front of intersection, respect traffic lights
			auto in_lane = v.conn.conn.a;

			auto cur_phase = node->traffic_light->decode_phase();
			auto signal_slot = node->traffic_light->_find_signal_slot(node, in_lane);
			auto lane_signal = node->traffic_light->get_signal(cur_phase, signal_slot);

			if (lane_signal == TrafficLight::RED) {
				// anything other than red means GO
				float dist = -v.front_k; // end of ingoing lane
				brake_for_dist(v.vehicle, dist);
				dbg_brake_for_blocked_lane_end(app, v, dist, in_lane);

				v.blocked = true;
			}
		}

		auto& avail_space = v.conn.conn.b.vehicles().avail_space;

		bool space_left = avail_space >= v.vehicle->vehicle_asset->car_len();

		// reserve space either if already on node or if on incoming lane and not blocked by traffic light
		if (!incoming_lane || (space_left && !v.blocked)) {
			// still reserve space even if none is avail if already on node
			avail_space -= v.vehicle->vehicle_asset->car_len() + SAFETY_DIST;
		}
		else {
			float dist = -v.front_k; // end of ingoing lane
			brake_for_dist(v.vehicle, dist);
			dbg_brake_for_blocked_lane_start(app, v, dist, v.conn.conn.b);

			v.blocked = true;
		}
	}

	// Remove vehicles completely off of intersection from list
	node->vehicles.test.remove_if([&] (NodeVehicle& v) {
		auto& state = v.vehicle->nav.get_state();
		if (state.motion == VehNav::END) {
			printf("");
		}

		float dist = v.front_k - v.conn.bez_len;
		return dist > v.vehicle->vehicle_asset->car_len();
	});

	int count = (int)node->vehicles.test.list.size();
	
	// Check each car against higher prio cars to yield to them
	for (int i=0; i<count; ++i) {
		auto& a = node->vehicles.test.list[i];
		
		if (a.vehicle->nav.get_state().get_cur_node() != node) {
			// already in outgoing lane, still yielding for other vehicles causes bug!
			continue;
		}

		//// loop over the last vehicle in each outgoing line
		//for (auto& seg : node->segments) {
		//	for (auto lane : seg->out_lanes(node)) {
		//		if (lane.vehicles().list.list.empty())
		//			continue;
		//		auto* v = lane.vehicles().list.list.back();
		//		NodeVehicle b = track_node_vehicle(v, v->path.get_state());
		//
		//		// compute intersection progress value 'k'
		//		update_ks(b);
		//		b.blocked = false;
		//
		//		if (b.rear_k >= b.conn.bez_len)
		//			continue;
		//
		//		bool dbg = (a.vehicle == sel || a.vehicle == sel2) && (b.vehicle == sel || b.vehicle == sel2);
		//		yield_for_car(app, node, a, b, dbg);
		//	}
		//}

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
			float b_rear_k = b->bez_t * b->bez_speed - b->vehicle_asset->car_len();

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

PosRot network::SimVehicle::calc_pos () {
	float3 dir = front_pos - rear_pos;
	float3 pos = front_pos - normalizesafe(dir) * vehicle_asset->car_len()*0.5f;
	float ang = angle2d((float2)dir);
	return { pos, ang };
}

void update_vehicle_suspension (Network& net, SimVehicle& vehicle, float3 local_accel, float dt) {
	// assume constant mass

	float3 ang = vehicle.suspension_ang;
	float3 vel = vehicle.suspension_ang_vel;

	// spring resitive accel
	//float2 accel = -ang * app.net.settings.suspension_spring_k;
	
	// quadratic for more smooth spring limit (and more wobbly around zero)
	float3 accel = -ang * abs(ang / net.settings.suspension.max) * net.settings.suspension.spring_k * 3;
	
	// spring point accel
	accel += local_accel * net.settings.suspension.accel_fac;
	// spring dampening
	accel -= vel * net.settings.suspension.spring_damp;

	// apply vel, pos and clamp
	vel += accel * dt;
	vel = clamp(vel, -100, +100);

	ang += vel * dt;
	ang = clamp(ang, -net.settings.suspension.max, +net.settings.suspension.max);

	vehicle.suspension_ang = ang;
	vehicle.suspension_ang_vel = vel;
}
void update_wheel_roll (SimVehicle& vehicle, float delta_dist, VehicleAsset* vehicle_asset) {
	float wheel_circum = vehicle_asset->wheel_r * (2*PI);

	vehicle.wheel_roll += delta_dist / wheel_circum;
	vehicle.wheel_roll = fmodf(vehicle.wheel_roll, 1.0f);
}

bool SimVehicle::update (Network& net, Metrics::Var& met, Person* driver, float dt) {
	auto& state = nav.get_state();
	if (state.motion == VehNav::END && bez_t >= state.end_t) {
		update_vehicle_suspension(net, *this, 0, dt); // update suspention lets it come to rest
		return true;
	}

	// bez_t == state.end_t can happen due to extrapolation between curves
	assert(bez_t <= 1.0f);
	
	float speed_limit = state.cur_speedlim;
	float aggress = driver->topspeed_accel_mul();
	{
		float remain_dist = (state.end_t - bez_t) * bez_speed;
		if (remain_dist <= 5.0f) {
			speed_limit = lerp(state.cur_speedlim, state.next_speedlim, map(remain_dist, 5.0f, 0.0f));
		}
		
		speed_limit *= aggress;
		speed_limit = max(speed_limit, 1.0f);
	}

	float old_speed = speed;
	float new_speed = old_speed;
	
	brake_light = 0.0f;

	// car speed change
	float target_speed = speed_limit * brake;
	if (target_speed < 0.33f) target_speed = 0;

	if (target_speed > new_speed) {
		float accel = aggress * calc_car_accel(net.settings.car_accel, speed_limit, new_speed);
		new_speed += accel * dt;
		new_speed = min(new_speed, target_speed);
	}
	else {
		//new_speed = target_speed; // brake instantly for now
		new_speed -= aggress * calc_car_deccel(net.settings.car_deccel, speed_limit, new_speed);
		new_speed = max(new_speed, target_speed);

		if (new_speed > 0.33f)
			brake_light = 1.0f;
	}

	speed = new_speed;
	met.total_flow += speed / speed_limit;
	
	// move car with speed on bezier based on previous frame delta t
	float delta_dist = speed * dt;
	bez_t += delta_dist / bez_speed;

	// do bookkeeping when car reaches end of current bezier
	if (bez_t >= state.end_t) {
		if (state.cur_vehicles) state.cur_vehicles ->remove(this);

		if (state.motion == VehNav::END) {
			// end path with bez_t still >= state.end_t, signalling done_navigating()
			bez_t = state.end_t;
		}
		else {
			float additional_dist = (bez_t - state.end_t) * bez_speed;
			bez_t = state.next_start_t;

			assert(bez_t >= 0 && bez_t < 1);

			nav.step(net);

			// NODE: what previously was next_vehicles (before step) is simply cur_vehicles after step, saving some memory
			if (state.cur_vehicles) state.cur_vehicles->add(this);

			// avoid visible jerk between bezier curves by extrapolating t
			auto start_bez_speed = length(state.bezier.eval(bez_t).vel);
			assert(additional_dist >= 0.0f);
			float additional_t = additional_dist / start_bez_speed;
			bez_t = min(bez_t + additional_t, state.end_t);

			{
				float blnk = 0;

				auto* node = state.get_cur_node();
				if (node) {
					auto turn = classify_turn(node, state.cur_lane.seg, state.next_lane.seg);
					if      (turn == Turns::LEFT ) blnk = -1;
					else if (turn == Turns::RIGHT) blnk = +1;
				}
				blinker = blnk;
			}
		}
	}

	// eval bezier at car front
	auto bez_res = state.bezier.eval_with_curv(bez_t);
	// remember bezier delta t for next frame
	bez_speed = length(bez_res.vel); // bezier t / delta pos
	float3 bez_dir = bez_res.vel / bez_speed;

	// actually move car rear using (bogus) trailer formula
	float3 new_front = bez_res.pos;
	
	float3 old_front = front_pos;
	float3 old_rear  = rear_pos;

	// I don't think this needs to be completely 3d
	auto moveDirs = relative2dir(normalizesafe(old_front - old_rear));
	//float forw_amount = dot(new_front - old_front, forw);

	float len = vehicle_asset->car_len();
	float3 ref_point = old_rear + len*net.settings.car_rear_drag_ratio * moveDirs.forw; // Kinda works to avoid goofy car rear movement?

	float3 new_rear = new_front - normalizesafe(new_front - ref_point) * len;

	front_pos = new_front;
	rear_pos  = new_rear;
	
	// totally wack with car_rear_drag_ratio
	turn_curv = bez_res.curv; // TODO: to be correct for wheel turning this would need to be computed based on the rear axle

	{
		float3 old_center = (old_front + old_rear) * 0.5f;
		float3 new_center = (new_front + new_rear) * 0.5f;
		float3 cen_vel   = dt == 0 ? 0 : (new_center - old_center) / dt;
		float3 cen_accel = dt == 0 ? 0 : (cen_vel - this->center_vel) / dt;
		
		// accel from world to local space
		//float accel_cap = 30; // we get artefacts with huge accelerations due to discontinuities, cap accel to hide
		//center_accel.y = clamp( dot(center_accel, right), -accel_cap, accel_cap);
		//center_accel.x = clamp( dot(center_accel, forw ), -accel_cap, accel_cap);
		float3 accel_local;
		accel_local.x = dot(cen_accel, moveDirs.right);
		accel_local.y = dot(cen_accel, moveDirs.forw );
		accel_local.z = dot(cen_accel, moveDirs.up   );
		update_vehicle_suspension(net, *this, -accel_local, dt);
		
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

		center_vel = float3(cen_vel, 0);

		update_wheel_roll(*this, delta_dist, vehicle_asset);
	}

	return false;
}

void Metrics::update (Var& var, App& app) {
	avg_flow = var.total_flow / (float)app.entities.persons.size();

	flow_plot.push_value(avg_flow);
}

bool VehicleTrip::start_trip (Entities& entities, Network& net, Random& rand, Person* person) {
	auto* target_building = entities.buildings[ rand.uniformi(0, (int)entities.buildings.size()) ].get();
		
	assert(person->cur_building->connected_segment);
	if (person->cur_building->connected_segment) {
		ZoneScopedN("pathfind");

		auto trip = std::make_unique<network::VehicleTrip>();
		
		trip->sim.vehicle_asset = person->owned_vehicle->asset;
		trip->sim.tint_col      = person->col;
		trip->sim.agressiveness = person->agressiveness;
		//trip->driver = person;
		trip->start  = from_vehicle_start(person->cur_building, *person->owned_vehicle);
		trip->target = Endpoint{ target_building, &target_building->parking_spot };

		bool valid = trip->sim.nav.pathfind(net, to_navpoint(trip->start), to_navpoint(trip->target));

		if (valid) {
			if (trip->start.parking) {
				auto* veh = (*trip->start.parking)->unpark();
				assert(veh == person->owned_vehicle.get());
			}

			person->cur_building = nullptr;
			person->owned_vehicle->state = std::move(trip);
			return true;
		}
	}
	return false;
}
void VehicleTrip::update_person (Entities& entities, Network& net, Metrics::Var& met, Random& rand, Person* person, float dt) {
	if (person->cur_building) {
		person->stay_timer += dt;
		if (person->stay_timer >= 3.0f) {
			person->stay_timer = 0;

			if (VehicleTrip::start_trip(entities, net, rand, person)) {
				dt = 0; // 0 dt timestep to init some values properly
			}
		}
	}

	auto* trip = person->owned_vehicle->get_trip();
	if (trip) {
		if (trip->update(net, met, person, dt)) {
			finish_trip(trip, person);
			return;
		}
		// trip ongoing
	}
}

void Network::simulate (App& app) {
	ZoneScoped;

	pathing_count = 0;

	float dt = app.sim_dt();

	// to avoid debugging overlays only showing while not paused, only skip moving the car when paused, later actually skip sim steps
	bool force_update_for_dbg = true;
	if (dt > 0.0f || force_update_for_dbg) {
		Metrics::Var met;
		
		{ // TODO: only iterate active vehicles
			ZoneScopedN("init pass");
			for (auto& pers : app.entities.persons) {
				//if (pers->cur_building) continue;
				auto* trip = pers->owned_vehicle->get_trip();
				if (trip)
					trip->sim.init();
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
				VehicleTrip::update_person(app.entities, app.network, met, app.sim_rand, person.get(), dt);
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
	ZoneScoped;

	debug_node(app, app.interact.selection.get<Node*>(), view);

	auto* pers = app.interact.selection.get<Person*>();
	if (pers && pers->owned_vehicle->get_trip()) {
		debug_vehicle(app, *pers->owned_vehicle, pers->owned_vehicle->get_trip()->sim, view);
	}

	debug_last_pathfind(app.network, view);
}

} // namespace network
