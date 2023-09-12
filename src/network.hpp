#pragma once
#include "common.hpp"
#include "assets.hpp"
#include "util.hpp"

struct Citizen;
struct Building;
struct App;

namespace network {
// TODO: naming
// path should be reserved for pathfinding
// general name to call a network edges? (road, pedestrian path, rail track etc.)
// how to call intersections? node seems fine

struct Node;
struct Segment;
struct Agent;
	
struct SegLane {
	Segment* seg;
	int      lane;

	inline bool operator== (SegLane const& r) const {
		return seg == r.seg && lane == r.lane;
	}
	inline bool operator!= (SegLane const& r) const {
		return seg != r.seg || lane != r.lane;
	}
};

struct Agent {
	Citizen* cit;

	float cur_t = 0;
	int   idx = 0;

	float _break;
		
	std::vector<Node*>   nodes;
	std::vector<SegLane> segments;

	Building* start = nullptr;
	Building* end   = nullptr;
};
struct AgentList { // TODO: optimize agents in lane to only look at agent in front of them, and speed up insert/erase by using linked list
	std::vector<Agent*> list;

	void add (Agent* agent) {
		assert(std::find(list.begin(), list.end(), agent) == list.end());
		list.push_back(agent);
	}
	void remove (Agent* agent) {
		auto it = std::find(list.begin(), list.end(), agent);
		assert(it != list.end());
		list.erase(it);
	}
};
struct SegAgents {
	std::vector<AgentList> lanes;
};
struct NodeAgents {
	AgentList free;
};

struct Node {
	float3 pos;
	float radius; // offset of segments
	// for editing and drawing?
	std::vector<Segment*> segments;

	// for Dijkstra, TODO: remove this from this data structure! indices instead of pointers needed to be able to have seperate node lists?
	// else always need to map pointers to other pointers or indices
	float    _cost;
	bool     _visited;

	Node*    _pred;
	SegLane  _pred_seg;

	NodeAgents agents;
	
	template <typename FUNC>
	void for_outgoing_lanes (FUNC func) const {
		for (auto* seg : segments) {
			int dir = this == seg->node_a ? 0 : 1;
			for (int i=0; i<(int)seg->layout->lanes.size(); ++i) {
				auto& lane = seg->layout->lanes[i];
				if (lane.direction == dir) {
					func(SegLane{ seg, i });
				}
			}
		}
	}
	template <typename FUNC>
	void for_ingoing_lanes (FUNC func) const {
		for (auto* seg : segments) {
			int dir = this == seg->node_a ? 1 : 0;
			for (int i=0; i<(int)seg->layout->lanes.size(); ++i) {
				auto& lane = seg->layout->lanes[i];
				if (lane.direction == dir) {
					func(SegLane{ seg, i });
				}
			}
		}
	}
	
	SelCircle get_sel_shape () {
		return { pos, radius, lrgb(0.04f, 0.04f, 1) };
	}
};

// Segments are oriented from a -> b, such that the 'forward' lanes go from a->b and reverse lanes from b->a
struct Segment { // better name? Keep Path and call path Route?
	RoadLayout* layout;
	Node* node_a;
	Node* node_b;

	float lane_length; // length of segment - node radii
	SegAgents agents;

	void update_cached () {
		lane_length = distance(node_a->pos, node_b->pos) - (node_a->radius + node_b->radius);
	}
		
	// Segment direction vectors
	struct Dirs {
		float2 forw, right;
	};
	Dirs clac_seg_vecs () {
		float2 ab = (float2)node_b->pos - (float2)node_a->pos;
		float2 forw = normalizesafe(ab);
		float2 right = rotate90(-forw); // cw rotate
		return { forw, right };
	}

	struct Line {
		float3 a, b;
	};
	Line clac_lane_info (int lane_i) {
		auto v = clac_seg_vecs();

		auto& lane = layout->lanes[lane_i];

		float3 a = node_a->pos + float3(v.right * lane.shift + v.forw * node_a->radius, 0);
		float3 b = node_b->pos + float3(v.right * lane.shift - v.forw * node_b->radius, 0);

		if (lane.direction == 0) return { a, b };
		else                     return { b, a };
	}
};

struct Network {
	std::vector<std::unique_ptr<Node>> nodes;
	std::vector<std::unique_ptr<Segment>> segments;
	
	float top_speed = 20;
	float cone = deg(45);

	void imgui () {
		ImGui::SliderFloat("top_speed", &top_speed, 0, 500);
		ImGui::SliderAngle("cone", &cone, 0, 180);
	}

	bool pathfind (Segment* start, Segment* target, Agent* path);

	int pathing_count;

	void simulate (App& app);
};

} // namespace network
