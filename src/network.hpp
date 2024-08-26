#pragma once
#include "common.hpp"
#include "assets.hpp"

struct Person;
struct Building;
class App;

namespace network {
// TODO: naming
// path should be reserved for pathfinding
// general name to call a network edges? (road, pedestrian path, rail track etc.)
// how to call intersections? node seems fine

inline constexpr float LANE_COLLISION_R = 1.3f;
inline constexpr float SAFETY_DIST = 1.0f;
// sidewalk is at 0, road actually gets lowered! TODO: config per road asset lane
inline constexpr float ROAD_Z = -0.15f;

struct Node;
struct Segment;
struct Lane;
struct ActiveVehicle;
struct LaneVehicles;
struct TrafficLight;

enum class Turns : uint8_t {
	NONE     = 0,

	LEFT     = 0b0001,
	STRAIGHT = 0b0010,
	RIGHT    = 0b0100,
	//UTURN    = 8,
	//CUSTOM     = 16,
	
	LS     = LEFT     | STRAIGHT,
	LR     = LEFT     | RIGHT,
	SR     = STRAIGHT | RIGHT,
	ALL    = LEFT     | STRAIGHT | RIGHT,
};
ENUM_BITFLAG_OPERATORS_TYPE(Turns, uint8_t)

struct Line {
	float3 a, b;
};
inline Bezier3<> calc_curve (Line const& l0, Line const& l1) {
	// Do this in 2d
	float2 point;
	if (!line_line_intersect((float2)l0.a, (float2)l0.b, (float2)l1.a, (float2)l1.b, &point))
		point = (l0.b+l1.a)*0.5f;
		//point = (l0.b+l1.a)*0.5f + 0.5f * distance(l0.a, l1.b) * normalizesafe(l0.b - l0.a);
					
	return { l0.b, float3(point, l0.a.z), l1.a };
}

// TODO: overhaul this! We need to track lanes per segment so we can actually just turn this into a pointer
struct SegLane {
	Segment* seg = nullptr;
	laneid_t lane = (laneid_t)-1;

	inline bool operator== (SegLane const& r) const {
		return seg == r.seg && lane == r.lane;
	}
	inline bool operator!= (SegLane const& r) const {
		return seg != r.seg || lane != r.lane;
	}

	operator bool () const { return seg != nullptr; }

	Lane& get ();
	
	Line clac_lane_info (float shift=0) const;
	
	LaneVehicles& vehicles () const;
};
VALUE_HASHER(SegLane, t.seg, t.lane);

struct Connection {
	SegLane a;
	SegLane b;
	
	// arbitrary order so we can treat b->a as a->b for Conflict cache
	bool operator< (Connection const& other) const {
		if      (a.seg  != other.a.seg ) return a.seg  < other.a.seg ;
		else if (b.seg  != other.b.seg ) return b.seg  < other.b.seg ;
		else if (a.lane != other.a.lane) return a.lane < other.a.lane; // Could combine lane < by merging ints
		else                             return b.lane < other.b.lane;
	}
	bool operator== (Connection const& other) const {
		return a == other.a && b == other.b;
	}
	bool operator!= (Connection const& other) const {
		return !(*this == other);
	}
};
VALUE_HASHER(Connection,
	t.a.seg, t.b.seg,
	hash_get_bits(t.a.lane, t.b.lane));

struct ConflictKey {
	Connection a, b;
	
	bool operator== (ConflictKey const& other) const {
		return a == other.a && b == other.b;
	}
	bool operator!= (ConflictKey const& other) const {
		return !(*this == other);
	}
};
VALUE_HASHER(ConflictKey,
	t.a.a.seg , t.a.b.seg , t.b.a.seg , t.b.b.seg ,
	hash_get_bits(t.a.a.lane, t.a.b.lane, t.b.a.lane, t.b.b.lane));

struct Conflict {
	float a_t0 = INF;
	float a_t1 = -INF;
	float b_t0 = INF;
	float b_t1 = -INF;

	operator bool () const { return a_t0 < INF; }
};

inline constexpr int COLLISION_STEPS = 4;

struct CachedConnection {
	Connection conn;
	float bez_len;
	
	float2 pointsL[COLLISION_STEPS+1];
	float2 pointsR[COLLISION_STEPS+1];
};

template <typename T>
struct VehicleList { // TODO: optimize vehicles in lane to only look at vehicle in front of them, and speed up insert/erase by using linked list
	std::vector<T> list;

	// TODO: can we avoid needing this?
	template <typename U>
	bool contains (U const& vehicle) {
		return std::find(list.begin(), list.end(), vehicle) != list.end();
	}

	void add (T vehicle) {
		assert(!contains(vehicle));
		list.push_back(vehicle);
	}
	template <typename U>
	void remove (U const& vehicle) {
		auto it = std::find(list.begin(), list.end(), vehicle);
		assert(it != list.end());
		list.erase(it);
	}

	template <typename FUNC>
	void remove_if (FUNC cond) {
		for (auto it=list.begin(); it!=list.end();) {
			if (cond(*it)) {
				it = list.erase(it);
			}
			else {
				it++;
			}
		}
	}
};

struct PathState {
	enum State { EXIT_BUILDING, ENTER_BUILDING, SEGMENT, NODE };

	State state; // type of motion
	float end_t = 1.0f;
	float next_start_t = 0.0f;
	Bezier3<> bezier;

	VehicleList<ActiveVehicle*>* cur_vehicles = nullptr;
	VehicleList<ActiveVehicle*>* next_vehicles = nullptr;

	SegLane cur_lane = {}; // always valid
	SegLane next_lane = {}; // only valid if cur_node != null

	Turns cur_turn = Turns::NONE;
};

struct ActiveVehicle {
//// Constants for entire trip (decided by game logic)
	Person* cit; // TODO: rename, later allow for cars with multiple passangers, in which case this would make sense anyway?

	// start and destination buildings
	Building* start = nullptr;
	Building* end   = nullptr;
	
//// Pathfinding result based on trip
	// all segments returned by pathfinding, without choosing lanes
	std::vector<Segment*> path;

//// Path following variables
	// current path sequence number (complicated, ideally this could be a generator function)
	// 0: start building -> road segment
	// 1: first segment, 2: first node, 3: next segment...
	// n-1: road segment -> end building
	int   idx = 0;

	float bez_t = 0; // [0,1] bezier parameter for current segment/node curve

	float brake = 1; // set by controlled conflict logic, to brake smoothly
	float speed = 0; // worldspace speed controlled by acceleration and brake

	// speed (delta position) / delta beizer t
	// INF to force no movement on initial tick (rather than div by 0)
	float bez_speed = INF; // set after timestep based on current bezier eval, to approx correct worldspace step size along bezier in next tick

	// cached info about current and future path following
	PathState state;
	
//// Movement sim variables for visuals
	float3 front_pos; // car front
	float3 rear_pos; // car rear

	// another velocity parameter, this time for the center of the car, to implement suspension
	// TODO: this should not exist, OR be the only velocity paramter
	float3 center_vel = 0;
	// suspension (ie. car wobble) angles based on car acceleration on forward and sideways axes (computed seperately)
	float3 suspension_ang = 0; // angle in radians, X: sideways (rotation on local X), Y: forwards
	float3 suspension_ang_vel = 0; // angular velocity in radians
	
	// curvature, ie. 1/turn_radius, positive means left
	float turn_curv = 0;
	float wheel_roll = 0;

	float blinker = 0;
	float blinker_timer = 0; // could get eliminated (a fixed number of blinker timers indexed using vehicle id hash)
	float brake_light = 0;

	bool update_blinker (float rand_num, float dt) {
		constexpr float blinker_freq_min = 1.6f;
		constexpr float blinker_freq_max = 1.2f;

		blinker_timer += dt * lerp(blinker_freq_min, blinker_freq_max, rand_num);
		blinker_timer = fmodf(blinker_timer, 1.0f);
		return blinker_timer > 0.5f;
	}

	float3 center () { return (front_pos + rear_pos)*0.5; };
	
	float car_len ();
	void calc_pos (float3* pos, float* ang);
};

struct NodeVehicle {
	ActiveVehicle* vehicle;

	int node_idx;
		
	// k == (approx) distance along node curve
	// where before node : k negative where abs(k) is dist to node
	// in node: k in [0, conn_len]
	// after node: k > conn_len where k-conn_len is dist from node
	float front_k;
	float rear_k;

	bool  blocked;
	float wait_time;

	CachedConnection conn;

	bool operator== (NodeVehicle const& other) const {
		return vehicle == other.vehicle;
	}
	bool operator== (ActiveVehicle* other) const {
		return vehicle == other;
	}
	template <typename U>
	bool operator!= (U const& other) const {
		return vehicle != other.vehicle;
	}
};

struct LaneVehicles {
	VehicleList<ActiveVehicle*> list;
	float avail_space;
};
struct SegVehicles {
	std::vector<LaneVehicles> lanes;
	//VehicleList<Vehicle*> free; // not part of lane, building->lane
};

struct NodeVehicles {
	VehicleList<ActiveVehicle*> free;

	VehicleList<NodeVehicle> test;

	// TODO: switch to that one flat hashmap i added to the project and profile?
	Hashmap<ConflictKey, Conflict, ConflictKeyHasher> conflict_cache;
};

struct Node {
	// node center
	float3 pos;

	// currently 'fake', it's simply the max offset of any connected segment to node center
	// NOTE: node center also does not really mean anything anymore now that segment ends can be positioned arbitrarily
	// TODO: should probably have node center (for editing) be some kind of bounding circle of all segment points
	float _radius;

	// for editing and drawing?
	// Sorted CCW in update_cached() for good measure
	std::vector<Segment*> segments;

	// for Dijkstra, TODO: remove this from this data structure! indices instead of pointers needed to be able to have seperate node lists?
	// else always need to map pointers to other pointers or indices
	float    _cost;
	bool     _visited;
	int      _q_idx;

	Node*    _pred;
	Segment* _pred_seg;

	bool _fully_dedicated_turns = false; // TODO: do this differently in the future
	
	std::unique_ptr<TrafficLight> traffic_light = nullptr;

	void set_traffic_light (bool has_traffic_light) {
		if (!has_traffic_light) {
			traffic_light = nullptr;
		}
		else {
			traffic_light = std::make_unique<TrafficLight>(this);
		}
	}
	void toggle_traffic_light () {
		set_traffic_light(traffic_light == nullptr);
	}

	NodeVehicles vehicles;
	
	void update_cached ();
	void set_defaults ();
	
	SelCircle get_sel_shape () {
		return { pos, _radius, lrgb(0.04f, 0.04f, 1) };
	}
};

struct Lane {
	Turns allowed_turns = Turns::ALL;
	bool  yield = false;
};

// Segments are oriented from a -> b, such that the 'forward' lanes go from a->b and reverse lanes from b->a
struct Segment { // better name? Keep Path and call path Route?
	NetworkAsset* asset;
	Node* node_a;
	Node* node_b;

	float3 pos_a;
	float3 pos_b;

	float _length = 0;

	SegVehicles vehicles;

	std::vector<Lane> lanes;
	laneid_t num_lanes () const { return (laneid_t)lanes.size(); }

	Lane& get_lane (NetworkAsset::Lane* layout_lane) {
		return lanes[layout_lane - &asset->lanes[0]];
	}
	NetworkAsset::Lane& get_lane_layout (Lane* lane) {
		return asset->lanes[lane - &lanes[0]];
	}
	
	Node* get_other_node (Node* node) {
		return node_a == node ? node_b : node_a;
	}
	LaneDir get_dir_to_node (Node* node) {
		return node_a != node ? LaneDir::FORWARD : LaneDir::BACKWARD;
	}
	LaneDir get_dir_from_node (Node* node) {
		return node_a == node ? LaneDir::FORWARD : LaneDir::BACKWARD;
	}
	
	Node* get_node_in_dir (LaneDir dir) {
		return dir == LaneDir::FORWARD ? node_b : node_a;
	}

	float3& pos_for_node (Node* node) {
		return node_a == node ? pos_a : pos_b;
	}

	// I despise iterators so much... I just want zero-cost generator functions....
	struct LanesRange {
		struct _Iter {
			SegLane sl;

			bool operator!= (_Iter const& other) {
				assert(sl.seg == other.sl.seg); // only happens if caller is stupid
				return sl.lane != other.sl.lane;
			}
			_Iter operator++ () {
				auto copy = *this;
				sl.lane++;
				return copy; // return copy that no one ever needs
			}
			SegLane operator* () {
				return sl; // don't actually deref anything
			}
		};

		Segment* seg;
		laneid_t first;
		laneid_t end_;

		_Iter begin () { return {{ seg, first }}; }
		_Iter end ()   { return {{ seg, end_ }}; }

		SegLane inner () { return { seg, first }; }
		SegLane outer () { return { seg, (laneid_t)(end_-1) }; }

		bool contains (SegLane sl) {
			return sl.seg == seg && (sl.lane >= first && sl.lane < end_);
		}

		SegLane operator[] (laneid_t idx) {
			assert(idx >= first && idx < end_);
			return { seg, (laneid_t)(first+idx) };
		}
	};
	LanesRange lanes_forward () {
		laneid_t count = (laneid_t)asset->num_lanes_in_dir(LaneDir::FORWARD);
		return { this, 0, count };
	}
	LanesRange lanes_backwards () {
		laneid_t first = (laneid_t)asset->num_lanes_in_dir(LaneDir::FORWARD);
		return { this, first, num_lanes() };
	}
	LanesRange lanes_in_dir (LaneDir dir) {
		return dir == LaneDir::FORWARD ? lanes_forward() : lanes_backwards();
	}
	LanesRange in_lanes (Node* node) {
		return lanes_in_dir(get_dir_to_node(node));
	}
	LanesRange out_lanes (Node* node) {
		return lanes_in_dir(get_dir_from_node(node));
	}

	void update_cached () {
		lanes.resize(asset->lanes.size());

		_length = distance(pos_a, pos_b);
	}
		
	// Segment direction vectors
	Dirs clac_seg_vecs () { // TODO: this code should go away
		float3 forw = normalizesafe(node_b->pos - node_a->pos);
		return relative2dir(forw);
	}
};
inline Line SegLane::clac_lane_info (float shift) const {
	auto v = seg->clac_seg_vecs();

	auto& l = seg->asset->lanes[lane];

	float3 seg_right  = v.right;
	float3 lane_right = l.direction == LaneDir::FORWARD ? v.right : -v.right;

	float3 a = seg->pos_a + seg_right * l.shift + lane_right * shift + float3(0,0,ROAD_Z);
	float3 b = seg->pos_b + seg_right * l.shift + lane_right * shift + float3(0,0,ROAD_Z);

	if (l.direction == LaneDir::FORWARD) return { a, b };
	else                                 return { b, a };
}
inline Lane& SegLane::get () {
	return seg->lanes[lane];
}

inline LaneVehicles& SegLane::vehicles () const {
	return seg->vehicles.lanes[lane];
}

// max lanes/segment and max segments per node == 256
inline uint32_t conn_id (Node* node, Connection const& conn) {
	int a_idx = indexof(node->segments, conn.a.seg); // TODO: optimize this? or just cache conn_id of cars instead?
	int b_idx = indexof(node->segments, conn.b.seg);

	uint32_t a = (uint32_t)a_idx | ((uint32_t)conn.a.lane << 8);
	uint32_t b = (uint32_t)b_idx | ((uint32_t)conn.b.lane << 8);

	return a | (b << 16);
}
inline uint64_t conn_pair_id (uint32_t conn_a_id, uint32_t conn_b_id) {
	return (uint64_t)conn_a_id | ((uint64_t)conn_b_id << 32);
}

struct Metrics {

	float avg_flow = 1; // avg of each (cur speed / cur speed limit)
	
	struct Var {
		float total_flow = 0;
	};
	
	void update (Var& var, App& app);

	ValuePlotter flow_plot = ValuePlotter();

	void imgui () {
		if (!imgui_Header("Metrics")) return;

		flow_plot.imgui_display("avg_flow", 0.0f, 1.0f);

		ImGui::PopID();
	}
};

struct Settings {
	SERIALIZE(Settings, car_accel, car_deccel, car_rear_drag_ratio, intersec_heur, suspension);
	
	float car_accel = 4.5f;
	float car_deccel = 5;

	float car_rear_drag_ratio = 0.4f;
	
	struct SuspensionVisuals {
		SERIALIZE(SuspensionVisuals, max, spring_k, spring_damp, accel_fac);

		float3 max = float3(deg(10), deg(6), 0.25f); // angle, angle, linear dist
		float3 spring_k = 50;
		float3 spring_damp = 5;
		float3 accel_fac = float3(0.2f, 0.2f, 0.2f);
	} suspension;

	struct IntersectionHeuristics {
		SERIALIZE(IntersectionHeuristics, wait_boost_fac, progress_boost, exit_eta_penal,
			right_before_left_penal, conflict_eta_penal, yield_lane_penal);
		
		float wait_boost_fac          = 1;
		float progress_boost          = 30;
		float exit_eta_penal          = 10;
		float right_before_left_penal = 15;
		float conflict_eta_penal      = 20;
		float yield_lane_penal        = 50;
	} intersec_heur;
	
	void imgui () {
		if (!imgui_Header("Network Settings")) return;
		
		ImGui::SliderFloat("car_accel (m/s^2)", &car_accel, 0, 20);
		ImGui::SliderFloat("car_deccel (m/s^2)", &car_deccel, 0, 20);

		ImGui::SliderFloat("car_rear_drag_ratio", &car_rear_drag_ratio, 0, 1);
		
		if (ImGui::TreeNode("Suspension Visuals")) {
			ImGui::SliderAngle("max.x", &suspension.max.x, 0, +30);
			ImGui::SliderAngle("max.y", &suspension.max.y, 0, +30);
			ImGui::SliderFloat("max.x", &suspension.max.z, 0, 4);
			ImGui::SliderFloat3("spring_k", &suspension.spring_k.x, 0, 100);
			ImGui::SliderFloat3("spring_damp", &suspension.spring_damp.x, 0, 100);
			ImGui::SliderFloat3("accel_fac", &suspension.accel_fac.x, 0, 10);
			ImGui::TreePop();
		}

		if (ImGui::TreeNode("Intersection Heuristics")) {
			ImGui::DragFloat("wait_boost_fac",          &intersec_heur.wait_boost_fac         , 0.1f);
			ImGui::DragFloat("progress_boost",          &intersec_heur.progress_boost         , 0.1f);
			ImGui::DragFloat("exit_eta_penal",          &intersec_heur.exit_eta_penal         , 0.1f);
			ImGui::DragFloat("right_before_left_penal", &intersec_heur.right_before_left_penal, 0.1f);
			ImGui::DragFloat("conflict_eta_penal",      &intersec_heur.conflict_eta_penal     , 0.1f);
			ImGui::DragFloat("yield_lane_penal",        &intersec_heur.yield_lane_penal       , 0.1f);
			ImGui::TreePop();
		}

		ImGui::PopID();
	}
};
struct Network {
	SERIALIZE(Network, settings);

	std::vector<std::unique_ptr<Node>> nodes;
	std::vector<std::unique_ptr<Segment>> segments;

	Metrics metrics;
	Settings settings;
	
	int _dijk_iter = 0;
	int _dijk_iter_dupl = 0;
	int _dijk_iter_lanes = 0;

	// Just an experiment for now
	float _random_lane_switching_chance = 0.15f;

	void imgui () {
		metrics.imgui();
		settings.imgui();

		ImGui::SliderFloat("random_lane_switching_chance", &_random_lane_switching_chance, 0, 1);
		_random_lane_switching_chance = clamp(_random_lane_switching_chance, 0.0f, 1.0f);
	}

	bool pathfind (Segment* start, Segment* target, ActiveVehicle* vehicle);

	int pathing_count;

	void simulate (App& app);
	void draw_debug (App& app, View3D& view);
};

inline Turns classify_turn (Node* node, Segment* in, Segment* out) {
	auto seg_dir_to_node = [] (Node* node, Segment* seg) {
		float2 dir = (float2)seg->clac_seg_vecs().forw; // dir: node_a -> node_b
		return seg->get_dir_to_node(node) == LaneDir::FORWARD ? dir : -dir;
	};

	float2 in_dir  = seg_dir_to_node(node, in);
	float2 out_dir = seg_dir_to_node(node, out);

	float2 rel;
	rel.y = dot(-out_dir, in_dir);
	rel.x = dot(-out_dir, rotate90(-in_dir));

	// TODO: track uturns?
	if (rel.y > abs(rel.x)) return Turns::STRAIGHT;
	else if (rel.x < 0.0f)  return Turns::LEFT;
	else                    return Turns::RIGHT;
}
inline bool is_turn_allowed (Node* node, Segment* in, Segment* out, Turns allowed) {
	return (classify_turn(node, in, out) & allowed) != Turns::NONE;
}

inline int default_lane_yield (int node_class, Segment& seg) {
	return seg.asset->road_class < node_class;
}
inline void set_default_lane_options (Node& node, bool fully_dedicated, int node_class) {
	
	for (auto& seg : node.segments) {
		auto dir = seg->get_dir_to_node(&node);
		int count = seg->asset->num_lanes_in_dir(dir);
		auto in_lanes = seg->lanes_in_dir(dir);
		
		if (count <= 1) {
			for (auto lane : in_lanes)
				lane.get().allowed_turns = Turns::ALL;
		}
		else if (count == 2 || !fully_dedicated) {
			for (auto lane : in_lanes) {
				bool leftmost  = lane.lane == in_lanes.first;
				bool rightmost = lane.lane == in_lanes.end_-1;
				if      (leftmost)  lane.get().allowed_turns = Turns::LS;
				else if (rightmost) lane.get().allowed_turns = Turns::SR;
				else                lane.get().allowed_turns = Turns::STRAIGHT;
			}
		}
		else {
			// fully dedicated
			int div = count / 3;
			int rem = count % 3;

			// equal lanes for left straight right turn, remainder goes to straight then left
			int L=div, S=div, R=div;
			if (rem >= 1) S+=1;
			if (rem >= 1) L+=1;

			for (auto lane : in_lanes) {
				int idx = 0;
				for (int i=0; i<L; ++i) in_lanes[idx++].get().allowed_turns = Turns::LEFT;
				for (int i=0; i<S; ++i) in_lanes[idx++].get().allowed_turns = Turns::STRAIGHT;
				for (int i=0; i<R; ++i) in_lanes[idx++].get().allowed_turns = Turns::SR; //Turns::RIGHT;
			}
		}

		for (auto lane : in_lanes) {
			lane.get().yield = default_lane_yield(node_class, *seg);
		}
	}
}

inline void Node::update_cached () {
	// Sort CCW(?) segments in place for good measure
	auto get_seg_angle = [] (Node* node, Segment* a) {
		Node* other = a->get_other_node(node);
		float2 dir = other->pos - node->pos;
		return atan2f(dir.y, dir.x);
	};
	std::sort(segments.begin(), segments.end(), [&] (Segment* l, Segment* r) {
		float ang_l = get_seg_angle(this, l);
		float ang_r = get_seg_angle(this, r);
		return ang_l < ang_r;
	});

	_radius = 0;
	for (auto* seg : segments) {
		_radius = max(_radius, distance(pos, seg->pos_for_node(this)));
	}
}
inline void Node::set_defaults () {
	int node_class;
	{
		node_class = 0;
		for (auto& seg : segments) {
			node_class = max(node_class, seg->asset->road_class);
		}
		
		auto want_traffic_light = [&] () {
			int non_small_segs = 0;
			for (auto& seg : segments) {
				if (seg->asset->road_class > 0)
					non_small_segs++;
			}

			// only have traffic light if more than 2 at least medium roads intersect
			return non_small_segs > 2;
		};
		set_traffic_light( want_traffic_light() );
	}

	set_default_lane_options(*this, _fully_dedicated_turns, node_class);
}

inline float get_cur_speed_limit (ActiveVehicle* vehicle) {
	auto state = vehicle->state.state;
	if (state == PathState::SEGMENT) {
		return vehicle->state.cur_lane.seg->asset->speed_limit;
	}
	else if (state == PathState::NODE) {
		float a = vehicle->state.cur_lane .seg->asset->speed_limit;
		float b = vehicle->state.next_lane.seg->asset->speed_limit;
		return min(a, b); // TODO: ??
	}
	else {
		return 20 / KPH_PER_MS;
	}
}

////
struct TrafficLightBehavior;
TrafficLightBehavior* default_TrafficLightBehavior (Node* node);

// One common state shared between all TrafficLightBehaviors, this is simpler than trying to allocate the correct state depending on behavoir
struct TrafficLight {
	//Node* node; // shouldn't need this ptr if we always update through node iteration
	TrafficLightBehavior* behavior;

	// could be a flat seconds, which loop after all cycles
	// or could map as cycle.progess (floori(timer) numer is cycle, fract(timer) is % progress)
	float timer = 0;

	TrafficLight (Node* node) {
		behavior = default_TrafficLightBehavior(node);
	}
};

enum class TrafficSignalState {
	//OFF = 0,
	RED = 0,
	YELLOW,
	GREEN,
};
// Allow shared bahavoirs (saves memory while allowing for different behaviors to be selected)
// TODO: By making this another type of asset, we should be able to vary easily allows custom behavoirs to be added and managed!
struct TrafficLightBehavior {
	static constexpr float yellow_time = 2.0f; // 1sec, TODO: make customizable or have it depend on duration of cycle?

	//enum Mode {
	//	SEGMENT_EXCLUSIVE,
	//};
	//Mode mode;

	float phase_go_duration; // how long we have green or yellow
	float phase_idle_duration; // how long to hold red before next phase starts

	int decode_phase (TrafficLight& state, float* green_remain) {
		int green_seg = floori(state.timer);
		float t = state.timer - (float)green_seg;

		float elapsed = t * (phase_go_duration + phase_idle_duration);
		*green_remain = phase_go_duration - elapsed;
		return green_seg;
	}

	void update (Node* node, float dt) {
		TrafficLight& state = *node->traffic_light;

		float num_seg = (float)node->segments.size();

		state.timer += dt / (phase_go_duration + phase_idle_duration);
		if (state.timer >= num_seg) {
			state.timer = fmodf(state.timer, num_seg);
		}
	}
	
	TrafficSignalState get_signal (Node* node, int seg_i, SegLane& lane) {
		TrafficLight& state = *node->traffic_light;
		
		float green_remain;
		int green_seg = decode_phase(state, &green_remain);

		if (green_seg == seg_i) {
			if (green_remain >= yellow_time)
				return TrafficSignalState::GREEN;
			if (green_remain >= 0.0f)
				return TrafficSignalState::YELLOW;
			// else green_remain < 0 -> phase_idle_duration, so everything red
		}
		return TrafficSignalState::RED;
	}

	void set_color (lrgb* R, lrgb* Y, lrgb* G, TrafficSignalState signal) {
		constexpr lrgb red    = lrgb(1,0,0); // TODO: make customizable
		constexpr lrgb yellow = lrgb(1,1,0);
		constexpr lrgb green  = lrgb(0,1,0);

		*R = signal == TrafficSignalState::RED ? red : lrgb(0);
		*Y = signal == TrafficSignalState::YELLOW ? yellow : lrgb(0);
		*G = signal == TrafficSignalState::GREEN ? green : lrgb(0);
	}

	// In order of segments, then in order of incoming lanes
	template <typename T>
	void push_signal_colors (Node* node, std::vector<T>& signal_colors) {
		for (int seg_i=0; seg_i<(int)node->segments.size(); ++seg_i) {
			auto& seg = node->segments[seg_i];

			for (auto in_lane : seg->in_lanes(node)) {
				auto state = get_signal(node, seg_i, in_lane);

				auto* colors = push_back(signal_colors, 1);
				set_color(&colors->colors[0], &colors->colors[1], &colors->colors[2], state);
			}
		}
	}
};

inline TrafficLightBehavior traffic_light_basic = { 10.0f, 2.0f };
inline TrafficLightBehavior* default_TrafficLightBehavior (Node* node) {
	return &traffic_light_basic;
}

} // namespace network
using network::Network;
