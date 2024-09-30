#pragma once
#include "common.hpp"
#include "assets.hpp"
#include "entities.hpp"

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

struct Network;
struct Node;
struct Segment;
struct Lane;
struct ActiveVehicle;
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
	ActiveVehicle* vehicle; // unnecessary

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
	//VehicleList<ActiveVehicle*> free;
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
	
	void update_cached ();
	void set_defaults ();
	
	SelCircle get_sel_shape () {
		return { pos, _radius, lrgb(0.04f, 0.04f, 1) };
	}

	Bezier3 calc_curve (Segment* seg0, Segment* seg1, float2 shiftXZ_0, float2 shiftXZ_1);
	Bezier3 calc_curve (SegLane& in, SegLane& out);
	
	static Node* between (Segment const* in, Segment const* out);

	int get_node_class ();
};

struct Lane {
	Turns allowed_turns = Turns::NONE;
	bool  yield = false;

	std::vector<SegLane> connections;
};

// Segments are oriented from a -> b, such that the 'forward' lanes go from a->b and reverse lanes from b->a
struct Segment { // better name? Keep Path and call path Route?
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

	float _length = 0;

	SegVehicles vehicles;

	std::vector<Lane> lanes;
	laneid_t num_lanes () const { return (laneid_t)lanes.size(); }

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
	}
};
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
inline Bezier3 Node::calc_curve (Segment* seg0, Segment* seg1, float2 shiftXZ_0, float2 shiftXZ_1) {
	auto i0 = seg0->get_end_info(this, shiftXZ_0);
	auto i1 = seg1->get_end_info(this, shiftXZ_1);

	float3 ctrl_in, ctrl_out;
	float2 point;
	// Find straight line intersection of in/out lanes with their tangents
	if (line_line_intersect((float2)i0.pos, (float2)i0.forw, (float2)i1.pos, (float2)i1.forw, &point)) {
		ctrl_in  = float3(point, i0.pos.z);
		ctrl_out = float3(point, i1.pos.z);
	}
	// Come up with seperate control points TODO: how reasonable is this?
	else {
		float dist = distance(i0.pos, i1.pos) * 0.5f;
		ctrl_in  = i0.pos + float3((float2)i0.forw, 0) * dist;
		ctrl_out = i1.pos + float3((float2)i1.forw, 0) * dist;
	}

	// NOTE: for quarter circle turns k=0.5539 would result in almost exactly a quarter circle!
	// https://pomax.github.io/bezierinfo/#circles_cubic
	// but turns that are sharper in the middle are more realistic, but we could make this customizable?
	float k = 0.6667f;

	Bezier3 bez;
	bez.a = i0.pos;
	bez.b = lerp(i0.pos, ctrl_in , k);
	bez.c = lerp(i1.pos, ctrl_out, k);
	bez.d = i1.pos;
	return bez;
}
inline Bezier3 Node::calc_curve (SegLane& in, SegLane& out) {
	// calc_curve works with shifts 
	auto& a0 = in.get_asset();
	auto& a1 = out.get_asset();
	float shift0 = a0.direction == LaneDir::FORWARD ? a0.shift : -a0.shift;
	float shift1 = a1.direction == LaneDir::FORWARD ? -a1.shift : a1.shift;

	return calc_curve(in.seg, out.seg, float2(shift0, ROAD_Z), float2(shift1, ROAD_Z));
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

class Pathfinding {
public:
	struct Endpoint {
		Segment* seg;
		// Able to pathfind while restricting the starting directions for example for repathing
		bool forw = true, backw = true;
	};

	static bool pathfind (Network& net, Endpoint start, Endpoint target,
		std::vector<Segment*>* result_path);
};

struct VehNavPoint {
	float3   pos;
	Segment* seg;
	// float t

	static bool free_point (Network const& net, float3 pos, VehNavPoint* result);
};

class VehNav {
public:
	enum MotionType { START, END, SEGMENT, NODE };

	struct State {
		// current path sequence number (complicated, ideally this could be a generator function)
		// 0: start building -> road segment
		// 1: first segment, 2: first node, 3: next segment...
		// n-1: road segment -> end building
		int idx = -1;
	
		MotionType motion;

		float end_t = 1.0f;
		float next_start_t = 0.0f;
		Bezier3 bezier;

		// if vehicle front either in incoming lane before node or in node:
		// cur_lane: incoming, next_lane: outgoing lane
		SegLane cur_lane;
		SegLane next_lane;

		// if vehicle front either in incoming lane before node or in node, else null
		Node* get_cur_node () const {
			if (cur_lane && next_lane)
				return Node::between(cur_lane.seg, next_lane.seg);
			return nullptr;
		}

		VehicleList<ActiveVehicle*>* cur_vehicles = nullptr;
		VehicleList<ActiveVehicle*>* next_vehicles = nullptr;
	};

private:
	// start and destination buildings
	VehNavPoint start;
	VehNavPoint target;
	
	// all segments returned by pathfinding, without choosing lanes
	// TODO: later this might have lane info (SegLane instead of Segment*) to allow for lane selection into the future
	// lane info is 1byte at most, so we can afford to store -1 for unchosen and 0-254 for chosen lanes
	// this enables tracking past lanes, which is good for tracking long vehicles that might cover more than 2 segments correctly (+ nodes inbetween)
	std::vector<Segment*> path;

	SegLane pick_lane (Network& net, Random& rand, int seg_i, SegLane prev_lane) const;
	
	State s;

	State _step (Network& net, SimVehicle* vehicle, int idx, State* prev_state) const;

public:
	State const& get_state () const { return s; }

	bool pathfind (Network& net, SimVehicle* vehicle, VehNavPoint const& start, VehNavPoint const& target);
	bool repath (Network& net, SimVehicle* vehicle, VehNavPoint const& new_target);

	void step (Network& net, SimVehicle* vehicle) {
		s = _step(net, vehicle, s.idx + 1, &s);
	}
	
	void visualize (OverlayDraw& overlay, Network& net, SimVehicle* vehicle,
		bool skip_next_node, lrgba col=lrgba(1,1,0,0.75f));

	// HACK: to fix problem with node vehicle tracking
	void _clear_nodes (SimVehicle* vehicle) {
		int num_seg = (int)path.size();
		
		for (int i=0; i<num_seg-1; ++i) {
			auto* node = Node::between(path[i], path[i+1]);
			node->vehicles.test.try_remove(vehicle);
		}
	}
};

struct SimVehicle {
	float bez_t = 0; // [0,1] bezier parameter for current segment/node curve

	float brake = 1; // set by controlled conflict logic, to brake smoothly
	float speed = 0; // worldspace speed controlled by acceleration and brake

	// speed (delta position) / delta beizer t
	// INF to force no movement on initial tick (rather than div by 0)
	float bez_speed = INF; // set after timestep based on current bezier eval, to approx correct worldspace step size along bezier in next tick

	VehNav nav;
	
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

	~SimVehicle () {
		auto& s = nav.get_state();
		if (s.cur_vehicles)  nav.get_state().cur_vehicles->try_remove(this);
		if (s.next_vehicles) assert(!s.next_vehicles->contains(this));


		nav._clear_nodes(this);
	}
};

class VehTrip {
	typedef NullableVariant<Building*, float3> Endpoint;

	Person* driver; // TODO: rename, later allow for cars with multiple passangers, in which case this would make sense anyway?
	
	Endpoint start;
	Endpoint target;
	
	SimVehicle sim;
};

////
struct TrafficLight {
	enum SignalState {
		//OFF = 0,
		RED = 0,
		YELLOW,
		GREEN,
	};

	static constexpr float yellow_time = 2.0f; // 1sec, TODO: make customizable or have it depend on duration of cycle?

	float phase_go_duration = 10; // how long we have green/yellow
	float phase_idle_duration = 2; // how long to hold red before next phase starts

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
	float _lane_switch_chance = 0.25f;

	void imgui () {
		metrics.imgui();
		settings.imgui();

		ImGui::SliderFloat("lane_switch_chance", &_lane_switch_chance, 0, 1);
		_lane_switch_chance = clamp(_lane_switch_chance, 0.0f, 1.0f);
	}

	int pathing_count;

	void simulate (App& app);
	void draw_debug (App& app, View3D& view);
	
	inline Segment* find_nearest_segment (float3 pos) const {
		Segment* nearest_seg = nullptr;
		float min_dist = INF;

		for (auto& seg : segments) {
			float dist = point_line_segment_dist(seg->pos_a, seg->pos_b - seg->pos_a, (float2)pos);
			if (dist < min_dist) {
				min_dist = dist;
				nearest_seg = seg.get();
			}
		}

		return nearest_seg;
	}
};

bool VehNavPoint::free_point (Network const& net, float3 pos, VehNavPoint* result) {
	result->seg = net.find_nearest_segment(pos);
	result->pos = pos;
	return result->seg != nullptr;
}

} // namespace network
using network::Network;
