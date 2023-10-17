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
struct LaneAgents;
	

struct Line {
	float3 a, b;
};

struct SegLane {
	Segment* seg;
	uint16_t lane;

	inline bool operator== (SegLane const& r) const {
		return seg == r.seg && lane == r.lane;
	}
	inline bool operator!= (SegLane const& r) const {
		return seg != r.seg || lane != r.lane;
	}
	
	Line clac_lane_info (float shift=0) const;

	LaneAgents& agents () const;
};
VALUE_HASHER(SegLane, t.seg, t.lane);

struct Connection {
	SegLane a, b;
	
	// arbitrary order so we can treat b->a as a->b for Conflict cache
	bool operator< (Connection const& other) const {
		if      (a.seg  != other.a.seg ) return a.seg  < other.a.seg ;
		else if (b.seg  != other.b.seg ) return b.seg  < other.b.seg ;
		else if (a.lane != other.a.lane) return a.lane < other.a.lane; // Could combine lane < by merging ints
		else                             return b.lane < other.b.lane;
	}
	bool operator== (Connection const& other) const {
		return a == other.a && b == other.b;
	}
	bool operator!= (Connection const& other) const {
		return !(*this == other);
	}
};
VALUE_HASHER(Connection,
	t.a.seg, t.b.seg,
	hash_get_bits(t.a.lane, t.b.lane));

struct ConflictKey {
	Connection a, b;
	
	bool operator== (ConflictKey const& other) const {
		return a == other.a && b == other.b;
	}
	bool operator!= (ConflictKey const& other) const {
		return !(*this == other);
	}
};
VALUE_HASHER(ConflictKey,
	t.a.a.seg , t.a.b.seg , t.b.a.seg , t.b.b.seg ,
	hash_get_bits(t.a.a.lane, t.a.b.lane, t.b.a.lane, t.b.b.lane));

struct Conflict {
	float a_t0 = INF;
	float a_t1 = -INF;
	float b_t0 = INF;
	float b_t1 = -INF;

	operator bool () const { return a_t0 < INF; }
};

inline constexpr int COLLISION_STEPS = 4;

struct CachedConnection {
	Connection conn;
	float bez_len;
	
	float2 pointsL[COLLISION_STEPS+1];
	float2 pointsR[COLLISION_STEPS+1];
};

struct Agent {
	Citizen* cit;

	int   idx = 0;

	float bez_t = 0;
	//float rear_t = 0; // only approximately correct
	
	std::vector<Node*>   nodes;
	std::vector<SegLane> segments;

	Building* start = nullptr;
	Building* end   = nullptr;

	float speed = 0; // real speed

	float bez_speed = 0; // delta beizer t over delta position
	
	float brake;
	bool  blocked;


};
template <typename T>
struct AgentList { // TODO: optimize agents in lane to only look at agent in front of them, and speed up insert/erase by using linked list
	std::vector<T> list;

	// TODO: can we avoid needing this?
	template <typename U>
	bool contains (U const& agent) {
		return std::find(list.begin(), list.end(), agent) != list.end();
	}

	void add (T agent) {
		assert(!contains(agent));
		list.push_back(agent);
	}
	template <typename U>
	void remove (U const& agent) {
		auto it = std::find(list.begin(), list.end(), agent);
		assert(it != list.end());
		list.erase(it);
	}

	template <typename FUNC>
	void remove_if (FUNC cond) {
		for (auto it=list.begin(); it!=list.end();) {
			if (cond(*it)) {
				it = list.erase(it);
			}
			else {
				it++;
			}
		}
	}
};

struct NodeAgent {
	Agent* agent;

	int node_idx;
		
	// k == (approx) distance along node curve
	// where before node : k negative where abs(k) is dist to node
	// in node: k in [0, conn_len]
	// after node: k > conn_len where k-conn_len is dist from node
	float front_k;
	float rear_k;

	CachedConnection conn;
	
	bool operator== (NodeAgent const& other) const {
		return agent == other.agent;
	}
	bool operator== (Agent* other) const {
		return agent == other;
	}
	template <typename U>
	bool operator!= (U const& other) const {
		return agent != other.agent;
	}
};

struct LaneAgents {
	AgentList<Agent*> list;
	float avail_space;
};
struct SegAgents {
	std::vector<LaneAgents> lanes;
};

struct NodeAgents {
	AgentList<Agent*> free;

	AgentList<NodeAgent> test;

	Hashmap<ConflictKey, Conflict, ConflictKeyHasher> conflict_cache;
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
	
	//template <typename FUNC>
	//void for_outgoing_lanes (FUNC func) const {
	//	for (auto* seg : segments) {
	//		int dir = this == seg->node_a ? 0 : 1;
	//		for (int i=0; i<(int)seg->layout->lanes.size(); ++i) {
	//			auto& lane = seg->layout->lanes[i];
	//			if (lane.direction == dir) {
	//				func(SegLane{ seg, i });
	//			}
	//		}
	//	}
	//}
	//template <typename FUNC>
	//void for_ingoing_lanes (FUNC func) const {
	//	for (auto* seg : segments) {
	//		int dir = this == seg->node_a ? 1 : 0;
	//		for (int i=0; i<(int)seg->layout->lanes.size(); ++i) {
	//			auto& lane = seg->layout->lanes[i];
	//			if (lane.direction == dir) {
	//				func(SegLane{ seg, i });
	//			}
	//		}
	//	}
	//}

	// TODO: can we get this info without needing so much memory
	std::vector<SegLane> in_lanes;
	std::vector<SegLane> out_lanes;
	
	// NOTE: this allows U-turns
	int num_conns () { return (int)in_lanes.size() * (int)out_lanes.size(); }
	
	void update_cached ();
	
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
};
inline Line SegLane::clac_lane_info (float shift) const {
	auto v = seg->clac_seg_vecs();

	auto& l = seg->layout->lanes[lane];

	float2 seg_right  = v.right;
	float2 lane_right = l.direction == 0 ? v.right : -v.right;

	float3 a = seg->node_a->pos + float3(seg_right * l.shift + lane_right * shift + v.forw * seg->node_a->radius, 0);
	float3 b = seg->node_b->pos + float3(seg_right * l.shift + lane_right * shift - v.forw * seg->node_b->radius, 0);

	if (l.direction == 0) return { a, b };
	else                  return { b, a };
}

inline LaneAgents& SegLane::agents () const {
	return seg->agents.lanes[lane];
}

inline void Node::update_cached () {
	for (auto* seg : segments) {
		int dir = this == seg->node_a ? 0 : 1; // 0: segment points 'away' from this node

		for (int i=0; i<(int)seg->layout->lanes.size(); ++i) {
			auto& lane = seg->layout->lanes[i];

			auto& vec = lane.direction == dir ? out_lanes : in_lanes;
			vec.push_back(SegLane{ seg, (uint16_t)i });
		}
	}
}

// max lanes/segment and max segments per node == 256
inline uint32_t conn_id (Node* node, Connection const& conn) {
	int a_idx = indexof(node->segments, conn.a.seg); // TODO: optimize this? or just cache conn_id of cars instead?
	int b_idx = indexof(node->segments, conn.b.seg);

	uint32_t a = (uint32_t)a_idx | ((uint32_t)conn.a.lane << 8);
	uint32_t b = (uint32_t)b_idx | ((uint32_t)conn.b.lane << 8);

	return a | (b << 16);
}
inline uint64_t conn_pair_id (uint32_t conn_a_id, uint32_t conn_b_id) {
	return (uint64_t)conn_a_id | ((uint64_t)conn_b_id << 32);
}

struct Network {
	std::vector<std::unique_ptr<Node>> nodes;
	std::vector<std::unique_ptr<Segment>> segments;
	
	SpeedUnit speed_unit = UNIT_KPH;

	float top_speed = 30 / KPH_PER_MS;
	float car_accel = 5;

	float rear_test = 0.4f;

	bool imgui_slider_speed (const char* label, float* speed, float min, float max) {
		float fac = SpeedUnitPerMs[speed_unit];

		top_speed *= fac;
		min *= fac;
		max *= fac;

		bool ret = ImGui::SliderFloat(
			prints("%s (%s)",label, SpeedUnitStr[speed_unit]).c_str(),
			speed, min, max);

		top_speed /= fac;
		return ret;
	}

	void imgui () {
		float speed_ms = top_speed;
		float speed_kmh = top_speed / 3.6f;

		ImGui::Combo("speed_unit", (int*)&speed_unit, SpeedUnitStr, ARRLEN(SpeedUnitStr));

		imgui_slider_speed("top_speed", &top_speed, 0, 200/KPH_PER_MS);

		ImGui::SliderFloat("car_accel (m/s^2)", &car_accel, 0, 20);

		ImGui::SliderFloat("rear_test", &rear_test, 0, 1);
	}

	bool pathfind (Segment* start, Segment* target, Agent* path);

	int pathing_count;

	void simulate (App& app);
};

} // namespace network
