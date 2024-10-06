#include "common.hpp"
#include "entities.hpp"
#include "network.hpp"

std::optional<PosRot> Vehicle::clac_pos () {
	return visit_overloaded(state,
		[] (std::monostate) -> std::optional<PosRot> {
			return std::nullopt; 
		},
		[] (std::unique_ptr<network::VehicleTrip> const& v) -> std::optional<PosRot> {
			return v->sim.calc_pos();
		},
		[] (ParkingSpot* const& v) -> std::optional<PosRot> {
			return v->calc_pos();
		}
	);
}
