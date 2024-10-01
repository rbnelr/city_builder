#include "common.hpp"
#include "entities.hpp"
#include "network.hpp"

SelCircle Person::get_sel_shape () { // TODO: move part of this to active vehicle?
	auto c = lrgb(0.04f, 1, 0.04f);
	if (trip)
		return { trip->sim.center(), trip->sim.car_len()*0.5f, c };
	return { cur_building->pos, 1, c };
}
