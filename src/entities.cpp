#include "common.hpp"
#include "entities.hpp"
#include "network.hpp"
#include "network_sim.hpp"

PosRot ParkingSpot::vehicle_center_pos (Vehicle* veh) const {
	float dist = size.y*0.5f - veh->asset->length()*0.5f;
	return { pos.local(float3(dist,0,0)), pos.ang };
}

float3 Person::calc_pos () {
	if (cur_building)
		return cur_building->pos;
	auto pos = owned_vehicle->calc_pos();
	assert(pos);
	return pos->pos;
}

Person::Person (Assets& assets, Random& rand, Building* initial_building) {
	cur_building = initial_building;

	owned_vehicle = Vehicle::create_random_vehicle(assets, rand);

	stay_timer = rand.uniformf(0,1);
}
