#pragma once
#include "common.hpp"
#include "network.hpp"

namespace network {

class Pathfinding {
public:
	struct Endpoint {
		Segment* seg;
		// Able to pathfind while restricting the starting directions for example for repathing
		bool forw = true, backw = true;
	};

	static bool pathfind (Network& net, Endpoint start, Endpoint dest,
		std::vector<Segment*>* result_path);
};

////

void update_segment (App& app, Segment* seg);
void update_node (App& app, Node* node, float dt);

struct NavEndCurve;

class IVehNav {
public:
	// start and destination getters implemented by Trip
	virtual NavEndCurve get_vehicle_trip_start (SegLane lane) = 0;
	// false: parking spot gets reserved on call, this call only happens just before dest is reached
	// visualize=true to avoid instantly reserving parking spot
	virtual NavEndCurve get_vehicle_trip_dest (SegLane lane, Vehicle& veh, bool visualize=false) = 0;
};

// Stores the path from pathfinding
// Is a Sequence of Motions that a vehicle performs to drive along a path
// step() should be called whenever the vehicle has performed the Motion represented by the current Motion
class VehNav {
public:
	void mem_use (MemUse& mem) {
		mem.add("VehNav", sizeof(*this));
		for (auto& i : path) mem.add("VehNav::path[]", sizeof(i));
	}

	virtual ~VehNav () {}

	enum MotionType { START, END, SEGMENT, NODE };

	struct Motion {
		// current path sequence number (complicated, ideally this could be a generator function)
		// 0: start building -> road segment
		// 1: first segment, 2: first node, 3: next segment...
		// n-1: road segment -> end building
		int idx = -1;
		
		MotionType motion;

		float end_t = 1.0f;
		float next_start_t = 0.0f;
		Bezier3 bezier;

		// Should this include slowdown during curves (and if so should it smoothly ramp down the speed somehow?)?
		float cur_speedlim; // speed limit on current lane
		float next_speedlim; // speed limit on next lane

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

		// Needed for correct Segment/Node updates
		LaneVehicles* cur_vehicles = nullptr;
	};

private:
	std::vector<Segment*> path;

	SegLane pick_lane (Network& net, Random& rand, int seg_i, SegLane prev_lane) const;
	
	Motion _step (Network& net, int idx, Motion* prev_state, Vehicle& veh, IVehNav* inav, bool visualize);

public:
	operator bool () const {
		return !path.empty();
	}

	bool pathfind (Motion& mot, Network& net, Vehicle& veh, Pathfinding::Endpoint start, Pathfinding::Endpoint dest, IVehNav* inav);
	bool repath (Motion& mot, Network& net, Vehicle& veh, Pathfinding::Endpoint new_dest, IVehNav* inav);

	void step (Motion& mot, Network& net, Vehicle& veh, IVehNav* inav) {
		mot = _step(net, mot.idx + 1, &mot, veh, inav, false);
	}
	
	void visualize (Motion& mot, OverlayDraw& overlay, Network& net, Vehicle& veh, IVehNav* inav,
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

// Stores all the data for Vehicle simulation, from traffic sim, to visual physics and other visuals
class SimVehicle {
public: // TODO: encapsulate better?

	// TODO: These are just copied from Person, can we avoid storing them twice?
	// Alternatively bundle them into Vehicle Instance* that both person and simVehicle use (note that parked cars also need vehicle_asset and col)
	VehicleAsset* vehicle_asset;
	lrgb tint_col;
	float agressiveness;

	VehNav::Motion mot;
	
	float bez_t = 0; // [0,1] bezier parameter for current segment/node curve

	float brake = 1; // set by controlled conflict logic, to brake smoothly
	float speed = 0; // worldspace speed controlled by acceleration and brake

	// speed (delta position) / delta beizer t
	// INF to force no movement on initial tick (rather than div by 0)
	// set after timestep based on current bezier eval, to approx correct worldspace step size along bezier in next tick
	float bez_speed = INF;

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

	void _init_pos (float3 pos, float3 forw) {
		front_pos = pos;
		rear_pos = pos - forw * vehicle_asset->length();
	}
	void init_pos (Bezier3 const& bez) {
		auto pos = bez.eval(0).pos;
		auto forw = normalizesafe(bez.eval(0.001f).pos - pos);
		_init_pos(pos, forw);
	}
	void init_pos (ParkingSpot* parking) {
		auto pos = parking->vehicle_front_pos();
		_init_pos(pos.pos, rotate3_Z(pos.ang) * float3(1,0,0));
	}

	bool update_blinker (float rand_num, float dt) {
		constexpr float blinker_freq_min = 1.6f;
		constexpr float blinker_freq_max = 1.2f;

		blinker_timer += dt * lerp(blinker_freq_min, blinker_freq_max, rand_num);
		blinker_timer = fmodf(blinker_timer, 1.0f);
		return blinker_timer > 0.5f;
	}

	float3 center () { return (front_pos + rear_pos)*0.5; };
	PosRot calc_pos ();
	
	virtual ~SimVehicle () {
		if (mot.cur_vehicles) mot.cur_vehicles->list.try_remove(this);
	}
	
	void begin_update () {
		brake = 1;
	}
	bool update (App& app, Network& net, Metrics::Var& met, VehNav& nav, Vehicle& veh, IVehNav* inav, float dt);
};

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

// Represents Start and Endpoint of Navigation
struct NavEndCurve {
	Bezier3 bez;
	float lane_t;

	struct Lane {
		SegLane const& lane;
		bool dir;
	};
	
	static float get_lane_t (SegLane const& lane, float lane_t) {
		float t = lane.get_asset().direction == LaneDir::FORWARD ? lane_t : 1.0f - lane_t;
		assert(t >= 0.0f && t <= 1.0f);
		return t;
	}

	static NavEndCurve calc_bezier (Lane const& lane, float3 pos, float3 ctrl, float lane_t) {
		auto lane_bez = lane.lane._bezier();
		float len = lane_bez.approx_len(4);
		float ctrl_t = min(3 / len, 0.5f); // control point 3m from actual nearest point

		float t0, t1;
		if (lane.dir == false) {
			t1 = clamp(get_lane_t(lane.lane, lane_t), 0.01f, 0.99f - ctrl_t); // limit such that t0 still in range
			t0 = t1 + ctrl_t;
		} else {
			t1 = clamp(get_lane_t(lane.lane, lane_t), 0.01f + ctrl_t, 0.99f); // limit such that t0 still in range
			t0 = t1 - ctrl_t;
		}
		assert(t0 > 0.0f && t0 < 1.0f);
		assert(t1 > 0.0f && t1 < 1.0f);
	
		float3 lane_ctrl = lane_bez.eval(t1).pos;
		float3 lane_pos  = lane_bez.eval(t0).pos;
		
		if (lane.dir == false)
			return { Bezier3(pos, ctrl, lane_ctrl, lane_pos), t0 };
		else
			return { Bezier3(lane_pos, lane_ctrl, ctrl, pos), t0 };
	}
	
	static NavEndCurve from_nearest_point_on_sidewalk (Lane const& lane, Segment* seg, float3 pos) {
		float t;
		seg->distance_to_point(pos, &t);
		
		auto bez = seg->bezier().eval(t);
		auto right = bez.dirs().right;

		float right_dist = dot(right, pos - bez.pos);
		float offset1 = right_dist > 0.0f ? seg->asset->sidewalkR : seg->asset->sidewalkL;
		float offset2 = right_dist > 0.0f ? seg->asset->sidewalkR+1 : seg->asset->sidewalkL-1;

		float3 end_pos = bez.pos + right * offset2;
		float3 ctrl    = bez.pos + right * offset1;

		return calc_bezier(lane, end_pos, ctrl, t);
	}

	//static NavEndPath building_viz (Lane const& lane, Building* build) {
	//	float t;
	//	float3 ctrl = PosRot(build->pos, build->rot).local(float3(-5, 0, 0));
	//	build->connected_segment->distance_to_point(build->pos, &t);
	//	return calc_bezier(lane, build->pos, ctrl, t);
	//}
	static NavEndCurve building_front (Lane const& lane, Building* build) { // for vehicles despawning "pocket cars"
		return from_nearest_point_on_sidewalk(lane, build->connected_segment, build->pos);
	}
	static NavEndCurve building_parking (Lane const& lane, Building* build, ParkingSpot* parking) {
		float3 pos = parking->vehicle_front_pos().pos;
		float3 ctrl = parking->front_enter_ctrl();
		float t;
		build->connected_segment->distance_to_point(ctrl, &t);
		return calc_bezier(lane, pos, ctrl, t);
	}
	static NavEndCurve street_parking (Lane const& lane, Building* build, ParkingSpot* parking) {
		float3 pos = parking->vehicle_front_pos().pos;
		float3 ctrl = lane.dir ? parking->side_enter_ctrl() : parking->side_exit_ctrl();
		float t;
		build->connected_segment->distance_to_point(ctrl, &t);
		return calc_bezier(lane, pos, ctrl, t);
	}
};
struct NavEndpoint {
	Building* building = nullptr;
	ParkingSpot* parking = {};

	static NavEndpoint from_vehicle_start (Building* start_building, Vehicle& veh) {
		auto* parking = std::get_if<ParkingSpot*>(&veh.state);
		if (parking) {
			return { start_building, *parking };
		}

		assert(std::get_if<std::monostate>(&veh.state)); // can't be in trip state
		return { start_building };
	}

	NavEndCurve calc_curve (NavEndCurve::Lane const& lane, bool visualize=false) const {
		//if (visualize)
		//	return NavEndCurve::building_viz(lane, building);

		if (parking) {
			bool is_building_parking = kiss::contains(building->parking, parking,
				[] (ParkingSpot const& l, ParkingSpot const* r) { return &l == r; });
			if (is_building_parking)
				return NavEndCurve::building_parking(lane, building, parking);
			else
				return NavEndCurve::street_parking(lane, building, parking);
		}
		else {
			return NavEndCurve::building_front(lane, building);
		}
	}
};

inline ParkingSpot* find_building_parking (Building* dest) {
	for (auto& spot : dest->parking) {
		if (spot.avail())
			return &spot;
	}
	return nullptr;
}
inline ParkingSpot* find_street_parking (Segment* seg) {
	for (auto& spot : seg->parking.spots) {
		if (spot.avail())
			return &spot;
	}
	return nullptr;
}
inline ParkingSpot* find_parking_near (Building* dest) {
	auto* spot = find_building_parking(dest);
	if (spot) return spot;

	return find_street_parking(dest->connected_segment);
}

// User of SimVehicle, currently represents a random trip from building to building
// More complexity TODO
class VehicleTrip : public IVehNav {
public:
	void mem_use (MemUse& mem) {
		mem.add("VehicleTrip", sizeof(*this));
		nav.mem_use(mem);
	}

	//Person* driver;

	SimVehicle sim;
	
	NavEndpoint start;
	NavEndpoint dest; // TODO: rename as dest?

	VehNav nav;
	
	~VehicleTrip () {
		nav._clear_nodes(&sim);

		// need to unreserve if deleted vehicle with trip etc.
		if (dest.parking)
			dest.parking->unreserve(this);
	}

// IVehNav
	NavEndCurve get_vehicle_trip_start (SegLane lane) override {
		return start.calc_curve({lane, false});
	}
	NavEndCurve get_vehicle_trip_dest (SegLane lane, Vehicle& veh, bool visualize=false) override {
		// find and reserve parking if not reserved yet, and do not reserve if just visualizing future path
		if (!dest.parking && !visualize) {
			dest.parking = find_parking_near(dest.building);
			if (dest.parking)
				dest.parking->reserve(&veh);
		}
		return dest.calc_curve({lane, true});
	}

	static void cancel_trip (VehicleTrip& trip, Person& person) {
		person.cur_building = trip.start.building;
		person.owned_vehicle->state = {}; // pocket car!
	}
	static void finish_trip (VehicleTrip& trip, Person& person) {
		ZoneScoped;
		
		person.cur_building = trip.dest.building;

		{ // handle vehicle
			if (trip.dest.parking)
				assert(trip.dest.parking->reserved_by(person.owned_vehicle.get()));
			if (trip.dest.parking && trip.dest.parking->reserved_by(person.owned_vehicle.get())) {
				auto* parking = trip.dest.parking;
				parking->park(person.owned_vehicle.get());
				person.owned_vehicle->state = parking; // WARNING: Destroys trip, do not evaluate  trip->dest.parking  on this line!
			}
			else {
				person.owned_vehicle->state = {};
			}
		}
	}

	static bool start_trip (Entities& entities, Network& net, Random& rand, Person& person) {
		auto* dest_building = entities.buildings[ rand.uniformi(0, (int)entities.buildings.size()) ].get();
		
		assert(person.cur_building->connected_segment);
		if (person.cur_building->connected_segment) {
			ZoneScoped;

			auto trip = std::make_unique<network::VehicleTrip>();
		
			trip->sim.vehicle_asset = person.owned_vehicle->asset;
			trip->sim.tint_col      = person.col;
			trip->sim.agressiveness = person.agressiveness;
			//trip->driver = person;
			trip->start = NavEndpoint::from_vehicle_start(person.cur_building, *person.owned_vehicle);
			trip->dest = NavEndpoint{ dest_building };

			bool valid = trip->nav.pathfind(trip->sim.mot, net, *person.owned_vehicle,
				Pathfinding::Endpoint{trip->start.building->connected_segment},
				Pathfinding::Endpoint{trip->dest.building->connected_segment},
				trip.get());

			if (valid) {
				if (trip->start.parking) {
					auto* veh = trip->start.parking->unpark();
					assert(veh == person.owned_vehicle.get());
					trip->sim.init_pos(trip->start.parking);
				}
				else {
					trip->sim.init_pos(trip->sim.mot.bezier);
				}

				person.cur_building = nullptr;
				person.owned_vehicle->state = std::move(trip);
				return true;
			}
		}
		return false;
	}
	static void update_person (App& app, Entities& entities, Network& net, Metrics::Var& met, Random& rand, Person& person, float dt) {
		if (person.cur_building) {
			// Person in building, wait for timer to start trip
			if (!wait_for(person.stay_timer, dt))
				return; // waiting

			if (!VehicleTrip::start_trip(entities, net, rand, person)) {
				assert(person.cur_building);
				person.stay_timer = 1;
				return; // start_trip failed
			}

			dt = 0; // 0 dt timestep to init some values properly
		}

		auto* trip = person.owned_vehicle->get_trip();
		assert(trip);

		
		net.active_vehicles++;
		if (!trip->sim.update(app, net, met, trip->nav, *person.owned_vehicle, trip, dt))
			return; // trip ongoing

		finish_trip(*trip, person);
		person.stay_timer = net._stay_time;
	}
};
inline void _mem_use (MemUse& mem, VehicleTrip* trip) {
	trip->mem_use(mem);
}

class DebugVehicles {
	std::vector<std::unique_ptr<Vehicle>> vehicles;


};

}; // namespace network
