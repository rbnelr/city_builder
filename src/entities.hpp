#pragma once
#include "common.hpp"
#include "assets.hpp"

namespace network {
	class Segment;
	class Node;
	class Vehicle;
	class PersonTrip;
}
class Building;
class Person;
using network::Vehicle;

// So we can bulldoze both personal vehicles and debug vehicles
// Might change later!
class IVehicleOwner {
public:
	virtual void remove_vehicle (Vehicle* vehicle) = 0;
};

// Should this class be a series of parking spots instead?
class ParkingSpot {
public:
	PosRot pos; // TODO: eliminate this
	float2 size;

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
	
	bool avail () {
		if (veh == nullptr) assert(!reserved);
		return veh == nullptr;
	}
	bool reserved_by (Vehicle* vehicle) {
		return reserved && veh == vehicle;
	}
	bool occupied_by (Vehicle* vehicle) {
		return !reserved && veh == vehicle;
	}
	
	void reserve (Vehicle* vehicle);
	void unreserve (Vehicle* vehicle);
	void park (Vehicle* vehicle);
	void unpark_keep_reserved (Vehicle* vehicle);

	// center
	PosRot calc_pos () const {
		return pos;
	}
	// 
	PosRot vehicle_front_pos () const {
		return { pos.local(float3(size.y*0.5f,0,0)), pos.ang };
	}
	PosRot vehicle_center_pos (Vehicle* veh) const;

	float3 front_enter_ctrl () const {
		return pos.local(float3(-size.y*1.5f,0,0));
	}
	float3 side_enter_ctrl () const {
		return pos.local(float3(-size.y*0.75f,+size.x*0.5f,0)); // backward left
	}
	float3 side_exit_ctrl () const {
		return pos.local(float3(+size.y*0.75f,+size.x*0.5f,0)); // forward left
	}

	void dbg_draw () {
		float3 F  = rotate3_Z(pos.ang) * float3(size.y,0,0);
		float3 R = rotate3_Z(pos.ang) * float3(0,-size.x,0);
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

class Building {
public:
	BuildingAsset* asset;

	float3 pos = 0;
	float  rot = 0;

	network::Segment* connected_segment = nullptr;

	std::vector<ParkingSpot> parking;
	static constexpr float2 PARKING_SPOT_SIZE = float2(2.8f, 5.2f);
	
	void update_cached (int num_parking) {
		float3 cur_pos = pos + rotate3_Z(rot) * float3(-3, 11, 0);
		float ang = rot;
		float3 dir = rotate3_Z(ang) * float3(0, 1, 0);

		for (int i=0; i<num_parking; ++i) {
			ParkingSpot spot;
			spot.pos = { cur_pos, ang };
			spot.size = PARKING_SPOT_SIZE;
			cur_pos += dir * spot.size.x;

			parking.push_back(spot);
		}
	}

	std::optional<SelCircle> get_sel_shape () {
		auto sz = asset->mesh.aabb.size();
		float radius = max(sz.x, sz.y) * 0.5f;
		return SelCircle{ pos, radius*0.5f, lrgb(1, 0.04f, 0.04f) };
	}

	void mem_use (MemUse& mem) {
		mem.add("Building", sizeof(*this));
		for (auto& i : parking) i.mem_use(mem);
	}
};

class Person : IVehicleOwner {
public:
	// TODO: needs to be some kind of state like in car, or in building
	// OR car/building etc needs to track citizen and we dont know where the citizen is
	// probably best to first use double pointers everywhere, likely that this is not a problem in terms of memory

	//Building* home = nullptr;
	//Building* work = nullptr;
	
	Building* cur_building = nullptr;
	float stay_timer = 1;

	std::unique_ptr<Vehicle> owned_vehicle;
	std::unique_ptr<network::PersonTrip> trip;

	//float agressiveness;

	Person (Assets& assets, Random& rand, Building* initial_building);

	float3 calc_pos ();

	//bool selectable () {
	//	// can only select while driving currently
	//	return owned_vehicle->clac_pos().has_value();
	//}
	//float3 get_sel_shape () {
	//	auto c = lrgb(0.04f, 1, 0.04f);
	//	if (cur_building)
	//		return { cur_building->pos, 1, c };
	//
	//	auto shape = owned_vehicle->get_sel_shape();
	//	assert(shape.has_value());
	//	return { shape->pos, shape->radius, c };
	//}

	void remove_vehicle (Vehicle* vehicle) override;
	
	void mem_use (MemUse& mem) {
		mem.add("Person", sizeof(*this));
		if (owned_vehicle) mem.add(*owned_vehicle);
		if (trip) mem.add(*trip);
	}
};

class Entities {
public:
	std::vector<std::unique_ptr<Building>> buildings;
	std::vector<std::unique_ptr<Person>> persons;

	// building and streets
	bool buildings_changed = true;
	
	void mem_use (MemUse& mem) {
		for (auto& i : buildings) i->mem_use(mem);
		for (auto& i : persons) i->mem_use(mem);
	}
};

typedef NullableVariant<Building*, /*Person*,*/ Vehicle*, network::Node*, network::Segment*> sel_ptr;
