#pragma once
#include "common.hpp"
#include "assets.hpp"
#include "entities.hpp"

namespace network {
// TODO: naming
// path should be reserved for pathfinding
// general name to call a network edges? (road, pedestrian path, rail track etc.)
// how to call intersections? node seems fine

inline constexpr float LANE_COLLISION_R = 1.3f;
inline constexpr float SAFETY_DIST = 1.0f;
// sidewalk is at 0, road actually gets lowered! TODO: config per road asset lane
inline constexpr float ROAD_Z = -0.15f;

struct Network;
class Node;
class Segment;
struct Lane;
class Vehicle;
struct LaneVehicles;
struct TrafficLight;

enum class Turns : uint8_t {
	NONE     = 0,

	STRAIGHT = 0b0001,
	RIGHT    = 0b0010,
	LEFT     = 0b0100,
	UTURN    = 0b1000,
	
	SR     = STRAIGHT | RIGHT,
	LS     = LEFT     | STRAIGHT,
	LR     = LEFT     | RIGHT,
	LSR    = LEFT     | STRAIGHT | RIGHT,
};
ENUM_BITFLAG_OPERATORS_TYPE(Turns, uint8_t)

// TODO: overhaul this! We need to track lanes per segment so we can actually just turn this into a pointer
struct SegLane {
	Segment* seg = nullptr;
	laneid_t lane = (laneid_t)-1;

	SegLane () {}
	SegLane (Segment* seg, laneid_t lane): seg{seg}, lane{lane} {}

	inline bool operator== (SegLane const& r) const {
		return seg == r.seg && lane == r.lane;
	}
	inline bool operator!= (SegLane const& r) const {
		return seg != r.seg || lane != r.lane;
	}
	
	operator bool () const {
		if (seg != nullptr) assert(lane >= 0);
		return seg != nullptr;
	}

	Lane& get () const;
	NetworkAsset::Lane& get_asset () const;

	Bezier3 _bezier () const;
	
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
	Connection conn; // This is unnecessary, can be read from PathState
	Bezier3 bezier;
	float bez_len; // This is related to the NodeVehicle k values, which are going to be changed soon
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
	// idx=-1 -> add, else insert at idx
	void insert (T vehicle, int idx) {
		assert(!contains(vehicle));
		assert(idx >= 0 && idx <= (int)list.size());
		list.insert(list.begin() + idx, vehicle);
	}
	
	template <typename U>
	void remove (U const& vehicle) {
		auto it = std::find(list.begin(), list.end(), vehicle);
		assert(it != list.end());
		list.erase(it);
	}
	template <typename U>
	bool try_remove (U const& vehicle) {
		auto it = std::find(list.begin(), list.end(), vehicle);
		if (it == list.end())
			return false;
		list.erase(it);
		return true;
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

// TODO: Might be able to eliminate this entirely! This would probably help perf and reduce complexity
struct NodeVehicle {
	void mem_use (MemUse& mem) {
		mem.add("NodeVehicle", sizeof(*this));
	}

	Vehicle* veh; // unnecessary

	// k == (approx) distance along node curve
	// where before node : k negative where abs(k) is dist to node
	// in node: k in [0, conn_len]
	// after node: k > conn_len where k-conn_len is dist from node

	// unnecessary since we want to compute these in one place for all the beziers a car touches
	// might cache on car relative to cur_lane/next_lane
	float front_k;
	float rear_k;

	// unnecessary, can be moved into ActiveVehicle, since these are only relevant for one node at a time
	// exception curretly happens when car straddles node/outgoing lane, but additional loop with _yield_for_car in update_node fixes this
	bool  blocked;
	float wait_time;

	// These are the only problem, since long vehicles might be involved in two nodes at once in terms of cars needing to yeild to conflict zones
	// solution might be to store cached connections in node next to conflict cache and do a double lookup
	// -> turns out the only non-trivial variable in CachedConnection are the cache points!, can also be eliminated!
	CachedConnection conn;

	// Turns out NodeVehicle is kinda not needed, we only need to track the order in the list for the prio order logic
	// But we could use ActiveVehicle* instead of NodeVehicle there (linked lists are also possible)
	// Even without the algorithm, we would still want a list of vehicles per intersection, for rendering purposes
	
	bool operator== (NodeVehicle const& other) const {
		return veh == other.veh;
	}
	bool operator== (Vehicle* other) const {
		return veh == other;
	}
	template <typename U>
	bool operator!= (U const& other) const {
		return veh != other.veh;
	}
};

struct LaneVehicles {
	void mem_use (MemUse& mem) {
		mem.add("LaneVehicles", sizeof(*this) + MemUse::sizeof_alloc(list.list));
	}

	VehicleList<Vehicle*> list;
	float avail_space;
	
	// find index in list of vehicles such all vehicles of indices below return value have
	// rear bezier t > bez_t
	// this is the correct index for insertion
	struct FindResult {
		int idx;
		Vehicle* leading = nullptr;
		Vehicle* trailing = nullptr;
	};
	FindResult find_lane_spot (float bez_t) const;

	void find_spot_and_insert (Vehicle* veh);
};
struct SegVehicles {
	void mem_use (MemUse& mem) {
		mem.add("SegVehicles", sizeof(*this));
		for (auto& i : lanes) i.mem_use(mem);
	}

	std::vector<LaneVehicles> lanes;
	//VehicleList<Vehicle*> free; // not part of lane, building->lane
};

struct NodeVehicles {
	void mem_use (MemUse& mem) {
		mem.add("NodeVehicles", sizeof(*this) + MemUse::sizeof_alloc(test.list));
		mem.add("NodeVehicles::Hashmap", MemUse::sizeof_alloc(conflict_cache));
	}

	//VehicleList<SimVehicle*> free;
	VehicleList<NodeVehicle> test;

	// TODO: switch to that one flat hashmap i added to the project and profile?
	Hashmap<ConflictKey, Conflict, ConflictKeyHasher> conflict_cache;
};

class Node {
public:
	void mem_use (MemUse& mem) {
		mem.add("Node", sizeof(*this));
		mem.add("Node::segments", MemUse::sizeof_alloc(segments));
		// TODO: traffic_light
		vehicles.mem_use(mem);
	}

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

	void replace_traffic_light (bool has_traffic_light) {
		if (!has_traffic_light) {
			traffic_light = nullptr;
		}
		else {
			traffic_light = std::make_unique<TrafficLight>(this);
		}
	}
	void toggle_traffic_light () {
		replace_traffic_light(traffic_light == nullptr);
	}

	NodeVehicles vehicles;
	
	void update_cached (float additional_node_radius);
	void set_defaults ();

	Bezier3 calc_curve (Segment* seg0, Segment* seg1, float2 shiftXZ_0, float2 shiftXZ_1);
	Bezier3 calc_curve (SegLane& in, SegLane& out);
	
	static Node* between (Segment const* in, Segment const* out);

	int get_node_class ();
	
	std::optional<SelCircle> get_sel_shape () {
		return SelCircle{ pos, _radius, lrgb(1, 0.5f, 0) };
	}
};

struct Lane {
	void mem_use (MemUse& mem) {
		mem.add("Lane", sizeof(*this) + MemUse::sizeof_alloc(connections));
	}

	Turns allowed_turns = Turns::NONE;
	bool  yield = false;

	std::vector<SegLane> connections;
};

class StreetParking {
public:
	std::vector<ParkingSpot> spots;
	//int forw_count;

	StreetParking () {}
	StreetParking (Segment* seg);

	void mem_use (MemUse& mem) {
		mem.add("StreetParking", sizeof(*this));
		for (auto& i : spots) i.mem_use(mem);
	}
};

// Segments are oriented from a -> b, such that the 'forward' lanes go from a->b and reverse lanes from b->a
class Segment { // better name? Keep Path and call path Route?
public:
	void mem_use (MemUse& mem) {
		mem.add("Segment", sizeof(*this));
		for (auto& i : lanes) i.mem_use(mem);
		vehicles.mem_use(mem);
		parking.mem_use(mem);
	}

	NetworkAsset* asset;
	Node* node_a;
	Node* node_b;

	float3 pos_a;
	float3 pos_b;

	float3 control_a () const {
		return lerp(pos_a, pos_b, 0.33333f); // TODO: implement curved segments!
	}
	float3 control_b () const {
		return lerp(pos_b, pos_a, 0.33333f);
	}
	float3 tangent_a () const { // return reverse of this, ie forward vector?
		return normalizesafe(control_a() - pos_a);
	}
	float3 tangent_b () const {
		return normalizesafe(control_b() - pos_b);
	}

	Bezier3 bezier () const {
		return Bezier3{ pos_a, control_a(), control_b(), pos_b };
	}
	Bezier3 _bezier_shifted (float2 shiftXZ) const;

	float distance_to_point (float3 pos, float* nearest_t=nullptr) const {
		// TODO: curved roads!
		return point_line_segment_dist((float2)pos_a, (float2)pos_b, (float2)pos, nearest_t);
	}

	float _length = 0;

	SegVehicles vehicles;

	std::vector<Lane> lanes;
	laneid_t num_lanes () const { return (laneid_t)lanes.size(); }

	StreetParking parking;

	Node* get_other_node (Node const* node) const {
		assert(node && (node == node_a || node == node_b));
		return node_a == node ? node_b : node_a;
	}
	Node* get_node_in_dir (LaneDir dir) const {
		return dir == LaneDir::FORWARD ? node_b : node_a;
	}
	
	LaneDir get_dir_to_node (Node const* node) const {
		assert(node && (node == node_a || node == node_b));
		return node_a != node ? LaneDir::FORWARD : LaneDir::BACKWARD;
	}
	LaneDir get_dir_from_node (Node const* node) const {
		assert(node && (node == node_a || node == node_b));
		return node_a == node ? LaneDir::FORWARD : LaneDir::BACKWARD;
	}

	static Node* node_from_lane (SegLane const& lane) {
		return lane.seg->get_node_in_dir(lane.get_asset().direction);
	}
	
	struct EndInfo {
		float3 pos;
		float3 forw;
		float3 right;
	};
	EndInfo get_end_info (LaneDir dir, float2 shiftXZ=0) {
		EndInfo i;
		if (dir == LaneDir::FORWARD) {
			i.pos = pos_b;
			i.forw = -tangent_b();
		}
		else {
			i.pos = pos_a;
			i.forw = -tangent_a();
		}
		i.right = rotate90_right(i.forw);
		i.pos += i.right * shiftXZ.x;
		i.pos += float3(0, 0, shiftXZ.y);
		return i;
	}
	EndInfo get_end_info (Node* node, float2 shiftXZ=0) {
		return get_end_info(get_dir_to_node(node), shiftXZ);
	}

	// I despise iterators so much... I just want zero-cost generator functions....
	
	// Lanes are iterated innermost to outermost in travel direction
	// for RHD: left to right
	struct LanesRange {
		struct _Iter {
			Segment* seg;
			int lane;

			bool operator!= (_Iter const& other) {
				assert(seg == other.seg); // only happens if caller is stupid
				return lane != other.lane;
			}
			_Iter operator++ () {
				auto copy = *this;
				lane++;
				return copy;
			}
			SegLane operator* () {
				return { seg, (laneid_t)lane }; // don't actually deref anything
			}
		};

		Segment* seg;
		laneid_t first;
		laneid_t end_;

		int count () {
			return (int)end_ - (int)first;
		}

		_Iter begin () { return { seg, first }; }
		_Iter end ()   { return { seg, end_ }; }

		SegLane inner (int idx=0) {
			assert(idx >= 0 && idx < count());
			return { seg, (laneid_t)(first+idx) };
		}
		SegLane outer (int idx=0) {
			assert(idx >= 0 && idx < count());
			return { seg, (laneid_t)(end_-1-idx) };
		}

		bool contains (SegLane sl) {
			return sl.seg == seg && (sl.lane >= first && sl.lane < end_);
		}

		SegLane operator[] (int idx) {
			return inner(idx);
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
	LanesRange in_lanes (Node const* node) {
		return lanes_in_dir(get_dir_to_node(node));
	}
	LanesRange out_lanes (Node const* node) {
		return lanes_in_dir(get_dir_from_node(node));
	}
	LanesRange all_lanes () {
		return { this, 0, num_lanes() };
	}

	void update_cached () {
		lanes.resize(asset->lanes.size());

		_length = distance(pos_a, pos_b);

		parking = StreetParking(this);
	}
	
	std::optional<SelRect> get_sel_shape () {
		float3 forw = tangent_a();
		float3 right = rotate90_right(forw);
		float3 pos = pos_a + right * asset->edgeL;
		forw *= _length;
		right *= asset->get_width();

		return SelRect{ pos, forw, right, lrgb(0, 0, 1) };
	}
};
inline StreetParking::StreetParking (Segment* seg) {
	auto create_parking_lane = [&] (float x, float2 spot_size, float ang_offs) {
		auto bez = seg->_bezier_shifted(float2(x,0));
		//float len = bez.approx_len(10);

		float spot_length = spot_size.y+0.3f;
		
		// one pass to find unallocated t
		float t = 0;
		auto prev_res = bez.eval(t);
		for (;;) {
			float step = prev_res.t_step(spot_length);
			if (t+step > 1.0f)
				break;
			t += step;
			prev_res = bez.eval(t);
		}

		// second pass to create spots centered on segment
		t = (1.0f - t) * 0.5f;
		prev_res = bez.eval(t);
		for (;;) {
			t += prev_res.t_step(spot_size.y+0.3f);
			if (t > 1.0f)
				break;

			auto res = bez.eval(t);

			ParkingSpot spot;
			spot.pos.pos = (prev_res.pos + res.pos) * 0.5f;
			float3 delta = res.pos - prev_res.pos;
			spot.pos.ang = angle2d((float2)delta) + ang_offs;
			spot.size = spot_size;

			spots.push_back(spot);

			prev_res = res;
		}
	};

	for (auto& l : seg->asset->parking) {
		create_parking_lane(l.shift, l.spot_size, l.direction == LaneDir::FORWARD ? 0 : deg(180));
	}
}

inline Node* Node::between (Segment const* in, Segment const* out) {
	if (in->node_a == out->node_a || in->node_a == out->node_b) {
		return in->node_a;
	}
	else {
		assert(in->node_b == out->node_a || in->node_b == out->node_b);
		return in->node_b;
	}
}
inline int Node::get_node_class () {
	int cls = 0;
	for (auto& seg : segments) {
		cls = max(cls, seg->asset->road_class);
	}
	return cls;
}

inline Lane& SegLane::get () const {
	assert(seg);
	return seg->lanes[lane];
}
inline NetworkAsset::Lane& SegLane::get_asset () const {
	assert(seg);
	return seg->asset->lanes[lane];
}
inline LaneVehicles& SegLane::vehicles () const {
	assert(seg);
	return seg->vehicles.lanes[lane];
}

inline Bezier3 Segment::_bezier_shifted (float2 shiftXZ) const {
	Bezier3 bez = bezier();
	float3 r0 = rotate90_right(tangent_a()); // This probably shouldn't include Z!
	float3 r1 = rotate90_right(-tangent_b());
	bez.a += r0 * shiftXZ.x + float3(0,0,shiftXZ.y);
	bez.b += r0 * shiftXZ.x + float3(0,0,shiftXZ.y);
	bez.c += r1 * shiftXZ.x + float3(0,0,shiftXZ.y);
	bez.d += r1 * shiftXZ.x + float3(0,0,shiftXZ.y);
	return bez;
}
// This math is dodgy because technically you can't offset a bezier by it's normal!
// TODO: implement curved segments and test this further!, perhaps a simple rule works good enough
inline Bezier3 SegLane::_bezier () const {
	assert(seg);

	float shift = get_asset().shift;
	auto bez = seg->_bezier_shifted(float2(shift, ROAD_Z));
	if (get_asset().direction == LaneDir::BACKWARD)
		bez = bez.reverse();
	return bez;
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

inline Turns classify_turn (Node* node, Segment* in, Segment* out) {
	auto seg_dir_to_node = [] (Node* node, Segment* seg) {
		return seg->get_dir_to_node(node) == LaneDir::FORWARD ?
			seg->tangent_b() : seg->tangent_a();
	};

	float2 in_dir  = seg_dir_to_node(node, in);
	float2 out_dir = seg_dir_to_node(node, out);

	float2 rel;
	rel.y = dot(-out_dir, in_dir);
	rel.x = dot(-out_dir, rotate90(-in_dir));

	float absX = abs(rel.x);
	if      ( rel.y > absX) return Turns::STRAIGHT;
	else if (-rel.y > absX) return Turns::UTURN;
	else if (rel.x < 0.0f)  return Turns::LEFT;
	else                    return Turns::RIGHT;
}
inline bool is_turn_allowed (Node* node, Segment* in, Segment* out, Turns allowed) {
	return (classify_turn(node, in, out) & allowed) != Turns::NONE;
}

////
struct TrafficLight {
	enum SignalState {
		//OFF = 0,
		RED = 0,
		YELLOW,
		GREEN,
	};

	static constexpr float yellow_time = 3; // 1sec, TODO: make customizable or have it depend on duration of cycle?

	float phase_go_duration = 30; // how long we have green/yellow
	float phase_idle_duration = 3; // how long to hold red before next phase starts

	float timer = 0;

	int num_phases = 0;
	// bitmasks of active lanes per phase, limit lanes in node to max 64
	std::unique_ptr<uint64_t[]> phases = nullptr;

	TrafficLight (Node* node);

	void update (Node* node, float dt) {
		timer += dt / (phase_go_duration + phase_idle_duration);
		timer = fmodf(timer, (float)num_phases);
	}

	struct CurPhase {
		uint64_t mask;
		float green_remain;
	};
	CurPhase decode_phase () {
		CurPhase res;

		int phase_i = floori(timer);
		float t = timer - (float)phase_i;
		
		float elapsed = t * (phase_go_duration + phase_idle_duration);

		res.mask  = phases[phase_i];
		res.green_remain = phase_go_duration - elapsed;
		return res;
	}
	SignalState get_signal (CurPhase& cur_phase, int signal_slot) {
		
		if (cur_phase.mask & ((uint64_t)1 << signal_slot)) {
			if (cur_phase.green_remain >= yellow_time)
				return GREEN;
			if (cur_phase.green_remain >= 0.0f)
				return YELLOW;
			// else green_remain < 0 -> phase_idle_duration, so everything red
		}
		return RED;
	}

	// In order of segments, then in order of incoming lanes
	template <typename T>
	void push_signal_colors (Node* node, std::vector<T>& signal_colors) {
		auto cur_phase = decode_phase();

		int signal_slot = 0;
		for (int seg_i=0; seg_i<(int)node->segments.size(); ++seg_i) {
			auto& seg = node->segments[seg_i];
			auto* light_asset = seg->asset->traffic_light_props.get();

			for (auto in_lane : seg->in_lanes(node)) {
				auto state = get_signal(cur_phase, signal_slot);

				auto* colors = push_back(signal_colors, 1);
				colors->colors[0] = state == RED    ? light_asset->colors[0] : lrgb(0);
				colors->colors[1] = state == YELLOW ? light_asset->colors[1] : lrgb(0);
				colors->colors[2] = state == GREEN  ? light_asset->colors[2] : lrgb(0);

				signal_slot++;
			}
		}
	}

	// usages probably should be optimized!
	static int _find_signal_slot (Node* node, SegLane& lane) {
		int signal_slot = 0;
		for (int seg_i=0; seg_i<(int)node->segments.size(); ++seg_i) {
			auto& seg = node->segments[seg_i];
			for (auto in_lane : seg->in_lanes(node)) {
				if (lane == in_lane)
					return signal_slot;
				signal_slot++;
			}
		}
		assert(false);
		return 0;
	}
};

struct Metrics {

	float avg_flow = 1; // avg of each (cur speed / cur speed limit)
	
	struct Var {
		float total_flow = 0;
		float total_flow_weight = 0;
	};
	
	void update (Var& var) {
		avg_flow = var.total_flow / var.total_flow_weight;

		flow_plot.push_value(avg_flow);
	}

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
			right_before_left_penal, conflict_eta_penal, yield_lane_penal, avoid_blocking_intersection);
		
		float wait_boost_fac          = 1;
		float progress_boost          = 30;
		float exit_eta_penal          = 10;
		float right_before_left_penal = 25;
		float conflict_eta_penal      = 15;
		float yield_lane_penal        = 50;

		bool avoid_blocking_intersection = true;
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
			
			ImGui::Checkbox("avoid_blocking_intersection", &intersec_heur.avoid_blocking_intersection);
			ImGui::TreePop();
		}

		ImGui::PopID();
	}
};

} // namespace network
using network::Network;
