#pragma once
#include "common.hpp"
#include "assets.hpp"

namespace network {
	struct Segment;
	struct Node;
	class VehicleTrip;

	void _mem_use (MemUse& mem, VehicleTrip* trip); // can't forward declare a method, but can a free function, bravo C++...
}
struct Building;
class Vehicle;
class ParkingSpot;

class Vehicle {
public:
	VehicleAsset* asset;
	lrgb col;
	
	// Could 
	typedef std::variant<
		std::monostate, // = Pocket car
		std::unique_ptr<network::VehicleTrip>,
		ParkingSpot*> State;
//private:
	State state;
//public:

	void mem_use (MemUse& mem) {
		mem.add("Vehicle", sizeof(*this));
		
		auto* trip = get_trip();
		if (trip) network::_mem_use(mem, trip);
	}

	network::VehicleTrip* get_trip () {
		auto* res = std::get_if<std::unique_ptr<network::VehicleTrip>>(&state);
		return res ? res->get() : nullptr;
	}

	Vehicle (Random& r, VehicleAsset* asset): asset{asset} {
		
		bool van = asset->mesh_filename == "vehicles/van.fbx";
		bool bus = asset->mesh_filename == "vehicles/bus.fbx";

		// 50% of cars (100% of busses) are colorful
		if (r.chance(0.5f) || bus) {
			col = hsv2rgb(r.uniformf(), 1.0f, 0.8f); // debug-like colorful colors
		}
		// other cars get black and white colors
		else {
			lrgb std_colors[] = {
				lrgb(0,0,0), // black
				lrgb(0,0,0), // 2nd black to make it more common
				lrgb(0.1f,0.1f,0.1f), // grey
				lrgb(0.5f,0.5f,0.55f), // silver?
				lrgb(1,1,1), // white
				lrgb(0.95f,0.1f,0.1f), // ?
			};
			col = std_colors[r.uniformi(0, ARRLEN(std_colors))];
		}

		// most transporters are white
		if (van && r.chance(0.8f)) {
			col = lrgb(1,1,1);
		}
	}
	~Vehicle ();

	std::optional<PosRot> clac_pos ();
	
	std::optional<SelCircle> get_sel_shape () {
		auto pos = clac_pos();
		if (pos)
			return SelCircle{ pos->pos, asset->length()*0.5f, 0 }; // return pos and radius for Person::get_sel_shape()
		return std::nullopt;
	}
};

// Should this class be a series of parking spots instead?
class ParkingSpot {
public:
	static constexpr float2 default_size = float2(2.8f, 5.2f);

	PosRot pos; // TODO: eliminate this

	// this probably is needed to be able to move vehicles when deleting segment/building,
	// but could be stored differently
	bool reserved = false; // true: not occupied yet, but not available   false: occupied
	Vehicle* veh = nullptr;

	void clear (Vehicle* vehicle) {
		if (veh == vehicle) {
			reserved = false;
			veh = nullptr;
		}
	}
	void unreserve (network::VehicleTrip* trip) {
		if (reserved && veh->get_trip() == trip)
			clear(veh);
	}
	
	bool avail () {
		if (veh == nullptr) assert(!reserved);
		return veh == nullptr;
	}
	bool reserved_by (Vehicle* vehicle) {
		return reserved && veh == vehicle;
	}
	
	void reserve (Vehicle* vehicle) {
		assert(avail());
		reserved = true;
		veh = vehicle;
	}
	void park (Vehicle* vehicle) {
		assert(avail() || reserved_by(vehicle));
		reserved = false;
		veh = vehicle;
	}
	Vehicle* unpark () {
		Vehicle* vehicle = veh;
		veh = nullptr;
		reserved = false;
		return vehicle;
	}

	// center
	PosRot calc_pos () const {
		return pos;
	}
	// 
	PosRot vehicle_front_pos () const {
		return { pos.local(float3(default_size.y*0.5f,0,0)), pos.ang };
	}
	PosRot vehicle_center_pos (Vehicle* veh) const {
		float dist = default_size.y*0.5f - veh->asset->length()*0.5f;
		return { pos.local(float3(dist,0,0)), pos.ang };
	}

	float3 front_enter_ctrl () const {
		return pos.local(float3(-default_size.y*1.5f,0,0));
	}
	float3 side_enter_ctrl () const {
		return pos.local(float3(-default_size.y*1.5f,+default_size.x*0.5f,0)); // backward left
	}
	float3 side_exit_ctrl () const {
		return pos.local(float3(+default_size.y*1.5f,+default_size.x*0.5f,0)); // forward left
	}

	void dbg_draw () {
		float3 F  = rotate3_Z(pos.ang) * float3(default_size.y,0,0);
		float3 R = rotate3_Z(pos.ang) * float3(0,-default_size.x,0);
		float3 p = pos.pos - F*0.5f - R*0.5f;

		float3 A = p;
		float3 B = p+R;
		float3 C = p+R+F;
		float3 D = p  +F;

		g_dbgdraw.line(A, B, lrgba(1,0.5f,0,1));
		g_dbgdraw.line(B, C, lrgba(1,0,0,1));
		g_dbgdraw.line(C, D, lrgba(1,0,0,1));
		g_dbgdraw.line(D, A, lrgba(1,0,0,1));
	}

	void mem_use (MemUse& mem) {
		mem.add("ParkingSpot", sizeof(*this));
	}
};

inline Vehicle::~Vehicle () {
	auto* parking = std::get_if<ParkingSpot*>(&state);
	if (parking) (*parking)->clear(this);
}

struct Building {
	BuildingAsset* asset;

	float3 pos = 0;
	float  rot = 0;

	network::Segment* connected_segment = nullptr;

	std::vector<ParkingSpot> parking;
	
	void update_cached (int num_parking) {
		float3 cur_pos = pos + rotate3_Z(rot) * float3(-5, 20, 0);
		float3 dir = rotate3_Z(rot) * float3(1, 0, 0);
		float ang = rot - deg(90);

		for (int i=0; i<num_parking; ++i) {
			ParkingSpot spot;
			spot.pos = { cur_pos, ang };
			cur_pos += dir * ParkingSpot::default_size.x;

			parking.push_back(spot);
		}
	}

	SelCircle get_sel_shape () {
		auto sz = asset->mesh.aabb.size();
		float radius = max(sz.x, sz.y) * 0.5f;
		return { pos, radius*0.5f, lrgb(1, 0.04f, 0.04f) };
	}

	void mem_use (MemUse& mem) {
		mem.add("Building", sizeof(*this));
		for (auto& i : parking) i.mem_use(mem);
	}
};

struct VehicleAgressiveness {
	static float get_random_for_person (Random& rand) {
		float agressiveness_deviation = 0.15f;
		return rand.normalf(agressiveness_deviation, 0.0f);
	}
	static float topspeed_accel_mul (float agressiveness) {
		return clamp(1.1f + agressiveness, 0.7f, 1.5f);
	}
};

struct Person {
	// TODO: needs to be some kind of state like in car, or in building
	// OR car/building etc needs to track citizen and we dont know where the citizen is
	// probably best to first use double pointers everywhere, likely that this is not a problem in terms of memory

	//Building* home = nullptr;
	//Building* work = nullptr;
	
	Building* cur_building = nullptr;
	float stay_timer = 1;

	std::unique_ptr<Vehicle> owned_vehicle;

	lrgb col;
	float agressiveness;

	Person (Random& r, Building* initial_building, lrgb col): cur_building{initial_building}, col{col} {
		float agressiveness_deviation = 0.15f;
		agressiveness = VehicleAgressiveness::get_random_for_person(r);

		stay_timer = r.uniformf(0,1);
	}

	float3 clac_pos () {
		if (cur_building)
			return cur_building->pos;
		auto pos = owned_vehicle->clac_pos();
		assert(pos); // while 
	}

	bool selectable () {
		// can only select while driving currently
		return owned_vehicle->clac_pos().has_value();
	}
	SelCircle get_sel_shape () {
		auto c = lrgb(0.04f, 1, 0.04f);
		if (cur_building)
			return { cur_building->pos, 1, c };

		auto shape = owned_vehicle->get_sel_shape();
		assert(shape.has_value());
		return { shape->pos, shape->radius, c };
	}
	
	void mem_use (MemUse& mem) {
		mem.add("Person", sizeof(*this));
		owned_vehicle->mem_use(mem);
	}
};

struct Entities {

	std::vector<std::unique_ptr<Building>> buildings;
	std::vector<std::unique_ptr<Person>> persons;

	// building and streets
	bool buildings_changed = true;
	
	void mem_use (MemUse& mem) {
		for (auto& i : buildings) i->mem_use(mem);
		for (auto& i : persons) i->mem_use(mem);
	}
};

typedef NullableVariant<Building*, Person*, network::Node*, network::Segment*> sel_ptr;
