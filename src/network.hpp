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

// TODO: overhaul this! We need to track lanes per segment so we can actually just turn this into a pointer
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

// extend the structs because otherwise I go crazy with nested structs and some contains searches break
struct InLane : public SegLane {
	bool yield;
};
struct OutLane : public SegLane {
	
};

struct Connection {
	InLane a;
	OutLane b;
	
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

	// curvature, ie. 1/turn_radius, positive means left
	float turn_curv = 0;

	float wheel_roll = 0;

	AgentState state;
	

	float3 front_pos;
	float3 rear_pos;

	float3 vel = 0;

	// suspension (ie. car wobble) angles based on car acceleration on forward and sideways axes (computed seperately)
	float2 suspension_ang = 0; // angle in radians, X: sideways (rotation on local X), Y: forwards
	float2 suspension_ang_vel = 0; // angular velocity in radians

	float3 center () { return (front_pos + rear_pos)*0.5; };
	
	float car_len ();
	void calc_pos (float3* pos, float* ang);
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
	float wait_time;

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
	// node center
	float3 pos;

	// currently 'fake', it's simply the max offset of any connected segment to node center
	// NOTE: node center also does not really mean anything anymore now that segment ends can be positioned arbitrarily
	// TODO: should probably have node center (for editing) be some kind of bounding circle of all segment points
	float _radius;

	// for editing and drawing?
	// Sorted CCW in update_cached() for good measure
	std::vector<Segment*> segments;

	// for Dijkstra, TODO: remove this from this data structure! indices instead of pointers needed to be able to have seperate node lists?
	// else always need to map pointers to other pointers or indices
	float    _cost;
	bool     _visited;
	int      _q_idx;

	Node*    _pred;
	Segment* _pred_seg;

	NodeAgents agents;
	
	// TODO: can we get this info without needing so much memory

	std::vector<InLane> in_lanes;
	std::vector<OutLane> out_lanes;
	
	// NOTE: this allows U-turns
	int num_conns () { return (int)in_lanes.size() * (int)out_lanes.size(); }
	
	InLane& get_in_lane (Segment* seg, uint16_t lane) {
		int idx = indexof(in_lanes, SegLane{ seg, lane });
		assert(idx >= 0);
		return in_lanes[idx];
	}
	
	void update_cached ();
	
	SelCircle get_sel_shape () {
		return { pos, _radius, lrgb(0.04f, 0.04f, 1) };
	}
};

struct Lane {
	AllowedTurns allowed_turns = ALLOWED_ALL;
};

// Segments are oriented from a -> b, such that the 'forward' lanes go from a->b and reverse lanes from b->a
struct Segment { // better name? Keep Path and call path Route?
	NetworkAsset* asset;
	Node* node_a;
	Node* node_b;

	float3 pos_a;
	float3 pos_b;

	float _length = 0;

	SegAgents agents;

	std::vector<Lane> lanes;

	Lane& get_lane (NetworkAsset::Lane* layout_lane) {
		return lanes[layout_lane - &asset->lanes[0]];
	}
	NetworkAsset::Lane& get_lane_layout (Lane* lane) {
		return asset->lanes[lane - &lanes[0]];
	}
	
	Node* get_other_node (Node* node) {
		return node_a == node ? node_b : node_a;
	}
	LaneDir get_dir_to_node (Node* node) {
		return node_b == node ? LaneDir::FORWARD : LaneDir::BACKWARD;
	}
	
	Node* get_node_in_dir (LaneDir dir) {
		return dir == LaneDir::FORWARD ? node_b : node_a;
	}

	float3 pos_for_node (Node* node) {
		return node_a == node ? pos_a : pos_b;
	}

	void update_cached () {
		lanes.resize(asset->lanes.size());

		_length = distance(pos_a, pos_b);
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

	auto& l = seg->asset->lanes[lane];

	float2 seg_right  = v.right;
	float2 lane_right = l.direction == LaneDir::FORWARD ? v.right : -v.right;

	float3 a = seg->pos_a + float3(seg_right * l.shift + lane_right * shift, 0);
	float3 b = seg->pos_b + float3(seg_right * l.shift + lane_right * shift, 0);

	if (l.direction == LaneDir::FORWARD) return { a, b };
	else                                 return { b, a };
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

		flow_plot.imgui_display("avg_flow", 0.0f, 1.0f);

		ImGui::TreePop();
	}
};

struct NetworkSettings {
	SERIALIZE(NetworkSettings, car_accel, car_deccel, car_rear_drag_ratio, intersec_heur);
	
	float car_accel = 4.5f;
	float car_deccel = 5;

	float car_rear_drag_ratio = 0.4f;
	
	float2 suspension_max_ang = float2(deg(10), deg(6));
	float2 suspension_spring_k = 50;
	float2 suspension_spring_damp = 5;
	float2 suspension_accel_fac = float2(0.2f, 0.2f);

	struct IntersectionHeuristics {
		SERIALIZE(IntersectionHeuristics, wait_boost_fac, progress_boost, exit_eta_penal,
			right_before_left_penal, conflict_eta_penal, yield_lane_penal);
		
		float wait_boost_fac          = 1;
		float progress_boost          = 30;
		float exit_eta_penal          = 10;
		float right_before_left_penal = 15;
		float conflict_eta_penal      = 20;
		float yield_lane_penal        = 50;
	};
	IntersectionHeuristics intersec_heur;
	
	void imgui () {
		if (!ImGui::TreeNodeEx("Network Settings", ImGuiTreeNodeFlags_DefaultOpen)) return;
		
		ImGui::SliderFloat("car_accel (m/s^2)", &car_accel, 0, 20);
		ImGui::SliderFloat("car_deccel (m/s^2)", &car_deccel, 0, 20);

		ImGui::SliderFloat("car_rear_drag_ratio", &car_rear_drag_ratio, 0, 1);
		
		ImGui::SliderAngle("suspension_max_ang.x", &suspension_max_ang.x, 0, +30);
		ImGui::SliderAngle("suspension_max_ang.y", &suspension_max_ang.y, 0, +30);
		ImGui::SliderFloat2("suspension_spring_k", &suspension_spring_k.x, 0, 100);
		ImGui::SliderFloat2("suspension_spring_damp", &suspension_spring_damp.x, 0, 100);
		ImGui::SliderFloat2("suspension_accel_fac", &suspension_accel_fac.x, 0, 10);

		if (ImGui::TreeNode("Intersection Heuristics")) {

			ImGui::DragFloat("wait_boost_fac",          &intersec_heur.wait_boost_fac         , 0.1f);
			ImGui::DragFloat("progress_boost",          &intersec_heur.progress_boost         , 0.1f);
			ImGui::DragFloat("exit_eta_penal",          &intersec_heur.exit_eta_penal         , 0.1f);
			ImGui::DragFloat("right_before_left_penal", &intersec_heur.right_before_left_penal, 0.1f);
			ImGui::DragFloat("conflict_eta_penal",      &intersec_heur.conflict_eta_penal     , 0.1f);
			ImGui::DragFloat("yield_lane_penal",        &intersec_heur.yield_lane_penal       , 0.1f);

			ImGui::TreePop();
		}

		ImGui::TreePop();
	}
};
struct Network {
	SERIALIZE(Network, settings);

	std::vector<std::unique_ptr<Node>> nodes;
	std::vector<std::unique_ptr<Segment>> segments;

	Metrics metrics;
	NetworkSettings settings;
	
	int _dijk_iter = 0;
	int _dijk_iter_dupl = 0;
	int _dijk_iter_lanes = 0;

	void imgui () {
		metrics.imgui();
		settings.imgui();
	}

	bool pathfind (Segment* start, Segment* target, Agent* path);

	int pathing_count;

	void simulate (App& app);
	void draw_debug (App& app, View3D& view);
};

inline void calc_default_allowed_turns (Node& node) {
	for (auto& in_lane : node.in_lanes) {
		auto& lane = in_lane.seg->lanes[in_lane.lane];

		auto dir = in_lane.seg->get_dir_to_node(&node);

		int count = in_lane.seg->asset->num_lanes_in_dir(dir);
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

inline int calc_node_class (Node& node) {
	int max_seg_class = 0;
	for (auto& seg : node.segments)
		max_seg_class = max(max_seg_class, seg->asset->road_class);
	return max_seg_class;
}

inline int default_lane_yield (int node_class, Segment& seg) {
	return seg.asset->road_class < node_class;
}

inline void Node::update_cached () {
	// Sort CCW(?) segments for good measure
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

	int node_class = calc_node_class(*this);

	_radius = 0;
	for (auto* seg : segments) {
		LaneDir dir = this == seg->node_a ? LaneDir::FORWARD : LaneDir::BACKWARD; // 0: segment points 'away' from this node

		for (int i=0; i<(int)seg->asset->lanes.size(); ++i) {
			auto& lane = seg->asset->lanes[i];

			if (lane.direction == dir) {
				out_lanes.push_back(OutLane{ seg, (uint16_t)i });
			}
			else {
				bool yield = default_lane_yield(node_class, *seg);
				in_lanes.push_back(InLane{ seg, (uint16_t)i, yield });
			}
		}

		_radius = max(_radius, distance(pos, seg->pos_for_node(this)));
	}

	calc_default_allowed_turns(*this);
}


} // namespace network
