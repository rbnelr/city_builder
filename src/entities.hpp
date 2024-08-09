#pragma once
#include "common.hpp"
#include "assets.hpp"
#include "network.hpp"

struct Building {
	BuildingAsset* asset;

	float3 pos = 0;
	float  rot = 0;

	network::Segment* connected_segment = nullptr;
};

struct Person {
	// TODO: needs to be some kind of state like in car, or in building
	// OR car/building etc needs to track citizen and we dont know where the citizen is
	// probably best to first use double pointers everywhere, likely that this is not a problem in terms of memory

	//Building* home = nullptr;
	//Building* work = nullptr;
	
	Building* cur_building = nullptr;
	float stay_timer = 0;

	// while driving this is non null and represents the currently driving car
	// (while inside building car does not exist)
	std::unique_ptr<network::ActiveVehicle> vehicle = nullptr;

	VehicleAsset* owned_vehicle;

	lrgb col;
	float agressiveness;

	float topspeed_accel_mul () {
		return clamp(1.1f + agressiveness, 0.7f, 1.5f);
	}

	Person (Random& r, Building* initial_building, VehicleAsset* owned_vehicle) {
		this->cur_building = initial_building;
		this->owned_vehicle = owned_vehicle;

		bool van = owned_vehicle->mesh_filename == "vehicles/van.fbx";
		bool bus = owned_vehicle->mesh_filename == "vehicles/bus.fbx";

		if (r.chance(0.5f) && !bus) {
			lrgb std_colors[] = {
				lrgb(0,0,0),
				lrgb(0,0,0),
				lrgb(0.1f,0.1f,0.1f),
				lrgb(0.5f,0.5f,0.55f),
				lrgb(1,1,1),
				lrgb(0.95f,0.1f,0.1f),
			};
			col = std_colors[r.uniformi(0, ARRLEN(std_colors))];
		}
		else {
			col = hsv2rgb(r.uniformf(), 1.0f, 0.8f); // debug-like colorful colors
		}

		if (van && r.chance(0.8f)) {
			col = lrgb(1,1,1);
		}

		float agressiveness_deviation = 0.15f;
		agressiveness = r.normalf(agressiveness_deviation, 0.0f);

		stay_timer = r.uniformf(0,1);
	}

	bool selectable () {
		// can only select while driving currently
		return vehicle != nullptr;
	}
	SelCircle get_sel_shape () {
		auto c = lrgb(0.04f, 1, 0.04f);
		if (vehicle)
			return { vehicle->center(), vehicle->car_len()*0.5f, c };
		return { cur_building->pos, 1, c };
	}
};

struct Entities {

	std::vector<std::unique_ptr<Building>> buildings;
	std::vector<std::unique_ptr<Person>> persons;

	// building and streets
	bool buildings_changed = true;
};
