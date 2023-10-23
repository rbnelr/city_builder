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

enum AllowedTurns : uint8_t {
	ALLOWED_LEFT     = 0b0001,
	ALLOWED_STRAIGHT = 0b0010,
	ALLOWED_RIGHT    = 0b0100,
	//ALLOWED_UTURN    = 8,
	//ALLOWED_CUSTOM     = 16,
	
	ALLOWED_LS     = ALLOWED_LEFT | ALLOWED_STRAIGHT,
	ALLOWED_LR     = ALLOWED_LEFT | ALLOWED_RIGHT,
	ALLOWED_SR     = ALLOWED_STRAIGHT | ALLOWED_RIGHT,
	ALLOWED_ALL    = ALLOWED_LEFT | ALLOWED_STRAIGHT | ALLOWED_RIGHT,
};
ENUM_BITFLAG_OPERATORS_TYPE(AllowedTurns, uint8_t)

struct Line {
	float3 a, b;
};
inline Bezier3 calc_curve (Line const& l0, Line const& l1) {
	float2 point;
	if (!line_line_intersect((float2)l0.a, (float2)l0.b, (float2)l1.a, (float2)l1.b, &point))
		point = (l0.b+l1.a)*0.5f;
		//point = (l0.b+l1.a)*0.5f + 0.5f * distance(l0.a, l1.b) * normalizesafe(l0.b - l0.a);
					
	return { l0.b, float3(point, l0.a.z), l1.a };
}

struct SegLane {
	Segment* seg = nullptr;
	uint16_t lane = (uint16_t)-1;

	inline bool operator== (SegLane const& r) const {
		return seg == r.seg && lane == r.lane;
	}
	inline bool operator!= (SegLane const& r) const {
		return seg != r.seg || lane != r.lane;
	}

	operator bool () const { return seg != nullptr; }
	
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

struct AgentState {
	enum State { EXIT_BUILDING, ENTER_BUILDING, SEGMENT, NODE };

	State state;
	float end_t = 1.0f;
	float next_start_t = 0.0f;
	Bezier3 bezier;
	float pos_z;

	network::AgentList<Agent*>* cur_agents = nullptr;
	network::AgentList<Agent*>* next_agents = nullptr;

	network::SegLane cur_lane = {}; // always valid
	network::Node*   cur_node = nullptr; // valid if != null
	network::SegLane next_lane = {}; // only valid if cur_node != null
};

struct Agent {
	Citizen* cit;

	int   idx = 0;

	float bez_t = 0;
	//float rear_t = 0; // only approximately correct
	
	std::vector<Segment*> path;

	Building* start = nullptr;
	Building* end   = nullptr;

	float speed = 0; // real speed

	// speed (delta position) / delta beizer t
	float bez_speed;

	float brake;

	AgentState state;
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

	bool  blocked;

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
	//AgentList<Agent*> free; // not part of lane, building->lane
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
	Segment* _pred_seg;

	NodeAgents agents;
	
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

struct Lane {
	AllowedTurns allowed_turns = ALLOWED_ALL;
};

// Segments are oriented from a -> b, such that the 'forward' lanes go from a->b and reverse lanes from b->a
struct Segment { // better name? Keep Path and call path Route?
	RoadLayout* layout;
	Node* node_a;
	Node* node_b;

	float lane_length; // length of segment - node radii
	SegAgents agents;

	std::vector<Lane> lanes;

	Lane& get_lane (RoadLayout::Lane* layout_lane) {
		return lanes[layout_lane - &layout->lanes[0]];
	}
	RoadLayout::Lane& get_lane_layout (Lane* lane) {
		return layout->lanes[lane - &lanes[0]];
	}
	
	Node* get_other_node (Node* node) {
		return node_a == node ? node_b : node_a;
	}
	LaneDir get_dir_to_node (Node* node) {
		return node_b == node ? DIR_FORWARD : DIR_BACKWARD;
	}

	void update_cached () {
		lane_length = distance(node_a->pos, node_b->pos) - (node_a->radius + node_b->radius);

		lanes.resize(layout->lanes.size());
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

struct Metrics {

	float avg_flow = 1; // avg of each (cur speed / cur speed limit)
	
	struct Var {
		float total_flow = 0;
	};
	
	void update (Var& var, App& app);

	ValuePlotter flow_plot = ValuePlotter();

	void imgui () {
		if (!ImGui::TreeNodeEx("Metrics", ImGuiTreeNodeFlags_DefaultOpen)) return;

		flow_plot.imgui_display("speed", 0.0f, 1.0f);

		ImGui::TreePop();
	}
};

struct Network {
	std::vector<std::unique_ptr<Node>> nodes;
	std::vector<std::unique_ptr<Segment>> segments;
	
	float car_accel = 5;

	float rear_test = 0.4f;

	Metrics metrics;

	void imgui () {
		ImGui::SliderFloat("car_accel (m/s^2)", &car_accel, 0, 20);

		ImGui::SliderFloat("rear_test", &rear_test, 0, 1);

		metrics.imgui();
	}

	bool pathfind (Segment* start, Segment* target, Agent* path);

	int pathing_count;

	void simulate (App& app);
};

inline void calc_default_allowed_turns (Node& node) {
	for (auto& in_lane : node.in_lanes) {
		auto& lane = in_lane.seg->lanes[in_lane.lane];

		auto dir = in_lane.seg->get_dir_to_node(&node);

		int count = in_lane.seg->layout->lanes_in_dir[dir];
		int idx   = in_lane.seg->get_lane_layout(&lane).order;

		if (count == 1) {
			lane.allowed_turns = ALLOWED_ALL;
		}
		else if (count == 2) {
			if (idx == 0) lane.allowed_turns = ALLOWED_LS;
			else          lane.allowed_turns = ALLOWED_SR;
		}
		else {
			lane.allowed_turns = ALLOWED_ALL;
		}
	}
}
inline AllowedTurns classify_turn (Node* node, Segment* in, Segment* out) {
	// TODO: store dir for seg end and start point?

	float2 in_dir = in->clac_seg_vecs().forw; // dir: node_a -> node_b
	if (in->node_a == node) in_dir = -in_dir;

	float2 out_dir = out->clac_seg_vecs().forw;
	if (out->node_a != node) out_dir = -out_dir;

	float d_forw  = dot(out_dir, in_dir);
	float d_right = dot(out_dir, rotate90(-in_dir));

	// TODO: track uturns?
	if (d_forw > abs(d_right)) return ALLOWED_STRAIGHT;
	else if (d_right < 0.0f)   return ALLOWED_RIGHT;
	else                       return ALLOWED_LEFT;
}
inline bool is_turn_allowed (Node* node, Segment* in, Segment* out, AllowedTurns allowed) {
	return (classify_turn(node, in, out) & allowed) != 0;
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

	calc_default_allowed_turns(*this);
}


} // namespace network
