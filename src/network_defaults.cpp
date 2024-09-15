#include "common.hpp"
#include "network.hpp"

namespace network {

int default_lane_yield (int node_class, Segment& seg) {
	return seg.asset->road_class < node_class;
}
void set_default_lane_options (Node& node, bool fully_dedicated, int node_class) {
	// TODO: rewrite to do this via integer counts instead of break?
	// Also might need better logic to handle things like major roads doing a turn!

	bool allow_mixed_lefts = node.traffic_light == nullptr;
	bool allow_mixed_rights = true;
	
	for (auto& seg : node.segments) {
		auto in_lanes = seg->in_lanes(&node);
		
		/*
		auto set_lane_arrows = [&] () {
			int count = in_lanes.count();

			if (count <= 1) {
				for (auto lane : in_lanes)
					lane.get().allowed_turns = Turns::LSR;
			}
			else if (count == 2 || !fully_dedicated) {
				for (auto lane : in_lanes) {
					bool leftmost  = lane.lane == in_lanes.first;
					bool rightmost = lane.lane == in_lanes.end_-1;
					if      (leftmost)  lane.get().allowed_turns = Turns::LS;
					else if (rightmost) lane.get().allowed_turns = Turns::SR;
					else                lane.get().allowed_turns = Turns::STRAIGHT;
				}
			}
			else {
				// fully dedicated
				int div = count / 3;
				int rem = count % 3;
			
				// equal lanes for left straight right turn, remainder goes to straight then left
				int L=div, S=div, R=div;
				if (rem >= 1) S+=1;
				if (rem >= 1) L+=1;
			
				for (auto lane : in_lanes) {
					int idx = 0;
					for (int i=0; i<L; ++i) in_lanes[idx++].get().allowed_turns = Turns::LEFT;
					for (int i=0; i<S; ++i) in_lanes[idx++].get().allowed_turns = Turns::STRAIGHT;
					for (int i=0; i<R; ++i) in_lanes[idx++].get().allowed_turns = Turns::SR; //Turns::RIGHT;
				}
			}

			for (auto lane : in_lanes) {
				lane.get().yield = default_lane_yield(node_class, *seg);
			}
		};
		*/
		auto set_connections = [&] () {
			
			std::vector<SegLane> outL, outS, outR;
			
			for (auto* seg_out : node.segments) {
				auto push_lanes = [&] (std::vector<SegLane>& vec) {
					for (auto lane : seg_out->out_lanes(&node)) {
						vec.push_back(lane);
					}
				};

				auto turn = classify_turn(&node, seg, seg_out);
				if      (turn == Turns::LEFT)     push_lanes(outL);
				else if (turn == Turns::STRAIGHT) push_lanes(outS);
				else if (turn == Turns::RIGHT)    push_lanes(outR);
			}
			
			int reqL = (int)outL.size();
			int reqS = (int)outS.size();
			int reqR = (int)outR.size();
			int avail_lanes = in_lanes.count();

			{
				int num = min(reqS, avail_lanes);
				for (int i=0; i<num; ++i) {
					auto& l = in_lanes.outer(i).get();
					l.connections.push_back(outS[reqS-1-i]);
					l.allowed_turns |= Turns::STRAIGHT;
				}
			}
			{
				int num = min(reqR, avail_lanes);
				for (int i=0; i<num; ++i) {
					auto& l = in_lanes.outer(i).get();
					bool mixed = any_set(l.allowed_turns, ~Turns::RIGHT);
					if (!mixed || allow_mixed_rights) {
						l.connections.push_back(outR[reqR-1-i]);
						l.allowed_turns |= Turns::RIGHT;
					}
					if (mixed) break;
				}
			}
			{
				int num = min(reqL, avail_lanes);
				for (int i=0; i<num; ++i) {
					auto& l = in_lanes.inner(i).get();
					bool mixed = any_set(l.allowed_turns, ~Turns::LEFT);
					if (!mixed || allow_mixed_lefts) {
						l.connections.push_back(outL[i]);
						l.allowed_turns |= Turns::LEFT;
					}
					if (mixed) break;
				}
			}
		};
		
		//set_lane_arrows();
		set_connections();
	}
}

void Node::update_cached () {
	// Sort CCW(?) segments in place for good measure
	auto get_seg_angle = [] (Node* node, Segment* a) {
		Node* other = a->get_other_node(node);
		float2 dir = other->pos - node->pos;
		return atan2f(dir.y, dir.x);
	};
	std::sort(segments.begin(), segments.end(), [&] (Segment* l, Segment* r) {
		float ang_l = get_seg_angle(this, l);
		float ang_r = get_seg_angle(this, r);
		return ang_l < ang_r;
	});

	_radius = 0;
	for (auto* seg : segments) {
		auto info = seg->get_end_info(seg->get_dir_to_node(this));
		_radius = max(_radius, distance(pos, info.pos));
	}

	// TODO: replace traffic light as well?
	// lane id for phases might have been invalidated!
}
void Node::set_defaults () {
	int node_class;
	{
		node_class = 0;
		for (auto& seg : segments) {
			node_class = max(node_class, seg->asset->road_class);
		}
		
		auto want_traffic_light = [&] () {
			int non_small_segs = 0;
			for (auto& seg : segments) {
				if (seg->asset->road_class > 0)
					non_small_segs++;
			}

			// only have traffic light if more than 2 at least medium roads intersect
			return non_small_segs > 2;
		};
		replace_traffic_light( want_traffic_light() );
	}

	set_default_lane_options(*this, _fully_dedicated_turns, node_class);
}

////
void setup_traffic_light_exclusive_segments (TrafficLight& light, Node* node) {
	light.num_phases = (int)node->segments.size();
	light.phases = std::make_unique<uint64_t[]>(light.num_phases);
	
	int signal_slot = 0;
	for (int phase_i=0; phase_i<light.num_phases; ++phase_i) {
		auto& seg = node->segments[phase_i];

		uint64_t mask = 0;

		for (auto in_lane : seg->in_lanes(node)) {
			mask |= (uint64_t)1 << signal_slot;
			signal_slot++;
		}

		light.phases[phase_i] = mask;
	}
}
void setup_traffic_light_2phase (TrafficLight& light, Node* node) {
	std::vector<uint64_t> phases;
	phases.reserve(8);
	auto create_phase = [&] (int active_seg1, int active_seg2) {
		uint64_t mask = 0;
		
		int signal_slot = 0;
		for (int seg_i=0; seg_i<(int)node->segments.size(); ++seg_i) {
			auto& seg = node->segments[seg_i];

			for (auto in_lane : seg->in_lanes(node)) {
				if (seg_i == active_seg1 || seg_i == active_seg2)
					mask |= (uint64_t)1 << signal_slot;
				signal_slot++;
			}
		}

		phases.push_back(mask);
	};

	std::vector<int> remain_segments;

	for (int i=0; i<(int)node->segments.size(); ++i)
		remain_segments.push_back(i);

	while (!remain_segments.empty()) {
		int seg = remain_segments[0];
		remain_segments.erase(remain_segments.begin());
		
		// take_straight_seg
		int straight_seg = -1;
		for (int j=0; j<(int)remain_segments.size(); ++j) {
			auto other_seg = remain_segments[j];

			auto turn = classify_turn(node, node->segments[seg], node->segments[other_seg]);
			if (turn == Turns::STRAIGHT) {
				straight_seg = other_seg;
				remain_segments.erase(remain_segments.begin()+j);
				break;
			}
		}

		create_phase(seg, straight_seg);
	}

	light.num_phases = (int)phases.size();
	light.phases = std::make_unique<uint64_t[]>(light.num_phases);
	memcpy(light.phases.get(), phases.data(), sizeof(phases[0])*light.num_phases);
}

TrafficLight::TrafficLight (Node* node) {
	//setup_traffic_light_exclusive_segments(*this, node);
	setup_traffic_light_2phase(*this, node);
}

} // namespace network
