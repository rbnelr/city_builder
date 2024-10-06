#pragma once
#include "common.hpp"
#include "assets.hpp"

namespace network {
	struct Segment;
	struct Node;
	class VehicleTrip;
}
struct Building;
class Vehicle;

// Should this class be a series of parking spots instead?
class ParkingSpot {
public:
	float3 pos; // TODO: eliminate this
	// this probably is needed to be able to move vehicles when deleting segment/building,
	// but could be stored differently
	Vehicle* parked_vehicle = nullptr;

	bool is_occupied () {
		return parked_vehicle;
	}
	void park (Vehicle* vehicle) {
		assert(!is_occupied());
		parked_vehicle = vehicle;
	}
	Vehicle* unpark () {
		Vehicle* vehicle = parked_vehicle;
		parked_vehicle = nullptr;
		return vehicle;
	}

	PosRot calc_pos () const {
		return { pos, 0 };
	}
};

class Vehicle {
public:
	VehicleAsset* asset;
	lrgb col;
	
	// allow pocket car if not otherwise possible for now?
	typedef std::variant<
		std::monostate, // = Pocket car? Might be needed for cases where no parking can be found! -> Apply No parking malus
		std::unique_ptr<network::VehicleTrip>,
		ParkingSpot*> State;
//private:
	State state;
//public:

	network::VehicleTrip* get_trip () {
		auto* res = std::get_if<std::unique_ptr<network::VehicleTrip>>(&state);
		return res ? res->get() : nullptr;
	}

	Vehicle (Random& r, VehicleAsset* asset): asset{asset} {
		
		bool van = asset->mesh_filename == "vehicles/van.fbx";
		bool bus = asset->mesh_filename == "vehicles/bus.fbx";

		// 50% of cars (100% of busses) are colorful
		if (r.chance(0.5f) && bus) {
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

	std::optional<PosRot> clac_pos ();
	
	std::optional<SelCircle> get_sel_shape () {
		auto pos = clac_pos();
		if (pos)
			return SelCircle{ pos->pos, asset->car_len()*0.5f, 0 }; // return pos and radius for Person::get_sel_shape()
		return std::nullopt;
	}
};

struct Building {
	BuildingAsset* asset;

	float3 pos = 0;
	float  rot = 0;

	network::Segment* connected_segment = nullptr;

	ParkingSpot parking_spot;
	
	void update_cached () {
		parking_spot.pos = pos + rotate3_Z(rot) * float3(-5, 20, 0);
	}

	SelCircle get_sel_shape () {
		auto sz = asset->mesh.aabb.size();
		float radius = max(sz.x, sz.y) * 0.5f;
		return { pos, radius*0.5f, lrgb(1, 0.04f, 0.04f) };
	}
};

struct Person {
	// TODO: needs to be some kind of state like in car, or in building
	// OR car/building etc needs to track citizen and we dont know where the citizen is
	// probably best to first use double pointers everywhere, likely that this is not a problem in terms of memory

	//Building* home = nullptr;
	//Building* work = nullptr;
	
	Building* cur_building = nullptr;
	float stay_timer = 0;

	std::unique_ptr<Vehicle> owned_vehicle;

	lrgb col;
	float agressiveness;

	float topspeed_accel_mul () {
		return clamp(1.1f + agressiveness, 0.7f, 1.5f);
	}

	Person (Random& r, Building* initial_building, lrgb col): cur_building{initial_building}, col{col} {
		float agressiveness_deviation = 0.15f;
		agressiveness = r.normalf(agressiveness_deviation, 0.0f);

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
};

struct Entities {

	std::vector<std::unique_ptr<Building>> buildings;
	std::vector<std::unique_ptr<Person>> persons;

	// building and streets
	bool buildings_changed = true;
};

typedef NullableVariant<Building*, Person*, network::Node*, network::Segment*> sel_ptr;
