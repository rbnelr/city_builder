#include "common.hpp"
#include "entities.hpp"
#include "network.hpp"
#include "network_sim.hpp"

void ParkingSpot::reserve (Vehicle* vehicle) {
	assert(avail() && !vehicle->parking);
	reserved = true;

	veh = vehicle;
	vehicle->parking = this;
}
void ParkingSpot::unreserve (Vehicle* vehicle) {
	assert(reserved_by(vehicle) && vehicle->parking == this);
	reserved = false;
	veh = nullptr;
	vehicle->parking = nullptr;
}
void ParkingSpot::park (Vehicle* vehicle) {
	assert((avail() && vehicle->parking == nullptr) ||
		   (reserved_by(vehicle) && vehicle->parking == this));
	reserved = false;
	veh = vehicle;
	vehicle->parking = this;
}
void ParkingSpot::unpark_keep_reserved (Vehicle* vehicle) {
	assert(occupied_by(vehicle) && vehicle->parking == this);
	reserved = true;
}

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

	owned_vehicle = std::make_unique<Vehicle>(Vehicle::create_random_vehicle(this, assets, rand));

	stay_timer = rand.uniformf(0,1);
}
void Person::remove_vehicle (Vehicle* vehicle) {
	if (trip) {
		trip->cancel_trip(*this);
	}
	else {
		vehicle->parking = nullptr; // vehicle goes to owner's pocket!
	}
}
