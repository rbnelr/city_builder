#pragma once
#include "common.hpp"
#include "engine/engine.hpp"
#include "game_camera.hpp"
#include "assets.hpp"

#include <queue>
#include <unordered_set>


struct BezierRes {
	float2 pos;
	float2 vel;   // velocity (over bezier t)
	float  curv;  // curvature (delta angle over dist along curve)
};
inline BezierRes bezier3 (float t, float2 a, float2 b, float2 c) {
	//float2 ab = lerp(a, b, t);
	//float2 bc = lerp(b, c, t);
	//return lerp(ab, bc, t);
		
	//float t2 = t*t;
	//
	//float _2t1 = 2.0f*t;
	//float _2t2 = 2.0f*t2;
	//
	//float ca = 1.0f -_2t1   +t2;
	//float cb =       _2t1 -_2t2;
	//float cc =               t2;
	//
	//return ca*a + cb*b + cc*c;

	float2 c0 = a;           // a
	float2 c1 = 2 * (b - a); // (-2a +2b)t
	float2 c2 = a - 2*b + c; // (a -2b +c)t^2
		
	float t2 = t*t;

	float2 value = c2*t2    + c1*t + c0; // f(t)
	float2 deriv = c2*(t*2) + c1;        // f'(t)
	float2 accel = c2*2;                 // f''(t)

		
	// angle of movement can be gotten via:
	// ang = atan2(deriv.y, deriv.x)

	// curvature can be defined as change in angle
	// atan2(deriv.y, deriv.x)
	// atan2 just offsets the result based on input signs, so derivative of atan2
	// should be atan(y/x)
		
	// wolfram alpha: derive atan(y(t)/x(t)) with respect to x:
	// gives me (x*dy - dx*y) / (x^2+y^2) (x would be deriv.x and dy would be accel.x)
	// this formula from the https://math.stackexchange.com/questions/3276910/cubic-b%c3%a9zier-radius-of-curvature-calculation?rq=1
	// seems to divide by the length of the sqrt(denom) as well, normalizing it by length(vel)
	// vel = dpos / t -> (x/dt) / (dpos/dt) -> x/dpos
	// so it seems this actually normalizes the curvature to be decoupled from your t (parameter) space
	// and always be correct in position space
	float denom = deriv.x*deriv.x + deriv.y*deriv.y;
	float curv = (deriv.x*accel.y - accel.x*deriv.y) / (denom * sqrt(denom)); // denom^(3/2)

	return { value, deriv, curv };
}
inline BezierRes bezier4 (float t, float2 a, float2 b, float2 c, float2 d) {
	//float2 ab = lerp(a, b, t);
	//float2 bc = lerp(b, c, t);
	//float2 cd = lerp(c, d, t);
	//
	//float2 abc = lerp(ab, bc, t);
	//float2 bcd = lerp(bc, cd, t);
	//
	//return lerp(abc, bcd, t);

	//float t2 = t*t;
	//float t3 = t2*t;
	//
	//float _3t1 = 3.0f*t;
	//float _3t2 = 3.0f*t2;
	//float _6t2 = 6.0f*t2;
	//float _3t3 = 3.0f*t3;
	//
	//float ca = 1.0f -_3t1 +_3t2   -t3;
	//float cb =       _3t1 -_6t2 +_3t3;
	//float cc =             _3t2 -_3t3;
	//float cd =                     t3;
	//
	//return (ca*a + cb*b) + (cc*c + cd*d);
		
	float2 c0 = a;                   // a
	float2 c1 = 3 * (b - a);         // (-3a +3b)t
	float2 c2 = 3 * (a + c) - 6*b;   // (3a -6b +3c)t^2
	float2 c3 = 3 * (b - c) - a + d; // (-a +3b -3c +d)t^3

	float t2 = t*t;
	float t3 = t2*t;
		
	float2 value = c3*t3     + c2*t2    + c1*t + c0; // f(t)
	float2 deriv = c3*(t2*3) + c2*(t*2) + c1;        // f'(t)
	float2 accel = c3*(t*6)  + c2*2;                 // f''(t)
		
	float denom = deriv.x*deriv.x + deriv.y*deriv.y;
	float curv = (deriv.x*accel.y - accel.x*deriv.y) / (denom * sqrt(denom)); // denom^(3/2)
		
	return { value, deriv, curv };
}

inline bool line_line_intersect (float2 a, float2 b, float2 c, float2 d, float2* out_point) {
	// https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
	float numer = (a.x-c.x)*(c.y-d.y) - (a.y-c.y)*(c.x-d.x);
	float denom = (a.x-b.x)*(c.y-d.y) - (a.y-b.y)*(c.x-d.x);
	if (denom == 0)
		return false; // parallel, either overlapping (numer == 0) or not
	float u = numer / denom;
	*out_point = a + u*(b-a);
	return true; // always intersect for now
}

struct App;

struct Renderer {

	virtual ~Renderer () {}
	
	virtual void imgui (App& app) = 0;

	virtual void begin (App& app) = 0;
	virtual void end (App& app) = 0;

	render::DebugDraw dbgdraw;
};

std::unique_ptr<Renderer> create_ogl_backend ();

struct Citizen;
struct Building;

struct Network {
	// TODO: naming
	// path should be reserved for pathfinding
	// general name to call a network edges? (road, pedestrian path, rail track etc.)
	// how to call intersections? node seems fine

	struct Node;
	struct Segment;
	struct Agent;

	struct Agent {
		struct Seg {
			Segment* seg;
			int      lane;
		};

		Citizen* cit;

		float cur_t = 0;
		int   idx = 0;
		
		std::vector<Node*> nodes;
		std::vector<Seg>   segments;

		Building* start = nullptr;
		Building* end   = nullptr;
	};
	struct Agents {
		//PathAgent* first_agent;
		std::vector< std::unordered_set<Agent*> > lanes;
		
		void add (Agent* agent, int lane) {
			lanes[lane].insert(agent);
		}
		void remove (Agent* agent, int lane) {
			assert(lanes[lane].find(agent) != lanes[lane].end());
			lanes[lane].erase(agent);
		}
	};

	struct Node {
		float3 pos;
		// for editing and drawing?
		std::vector<Segment*> segments;

		// for Dijkstra, TODO: remove this from this data structure! indices instead of pointers needed to be able to have seperate node lists?
		// else always need to map pointers to other pointers or indices
		float    _cost;
		bool     _visited;
		Node*    _pred;
		Segment* _pred_seg;
		int      _pred_lane;
	};

	// Segments are oriented from a -> b, such that the 'forward' lanes go from a->b and reverse lanes from b->a
	struct Segment { // better name? Keep Path and call path Route?
		RoadLayout* layout;
		Node* node_a;
		Node* node_b;

		Agents agents;

		float calc_length () {
			return distance(node_a->pos, node_b->pos);
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
			float offset = layout->width * 0.5f; // offset of segment back from node center, ie "size" of intersection

			float3 a = node_a->pos + float3(v.right * lane.shift + v.forw * offset, 0);
			float3 b = node_b->pos + float3(v.right * lane.shift - v.forw * offset, 0);

			if (lane.direction == 0) return { a, b };
			else                     return { b, a };
		}
	};

	std::vector<std::unique_ptr<Node>> nodes;
	std::vector<std::unique_ptr<Segment>> segments;

	bool pathfind (Segment* start, Segment* target, Agent* path) {
		// use dijstra

		struct Comparer {
			bool operator () (Node* l, Node* r) {
				// true: l < r
				// but since priority_queue only lets us get
				// max element rather than min flip everything around
				return l->_cost > r->_cost;
			}
		};
		std::priority_queue<Node*, std::vector<Node*>, Comparer> unvisited;
		
		// queue all nodes
		for (auto& node : nodes) {
			node->_cost = INF;
			node->_visited = false;
			node->_pred = nullptr;
		}

		// TODO: could actually start only on the lane facing the target building
		//  (ie. don't allow crossing the road middle)
		{ // handle the two start nodes
			auto len = start->calc_length();
			start->node_a->_cost = len / 0.5f; // pretend start point is at center of start segment for now
			start->node_b->_cost = len / 0.5f;
		}

		unvisited.push(start->node_a);
		unvisited.push(start->node_b);

		while (!unvisited.empty()) {
			// visit node with min distance
			Node* cur_node = unvisited.top();
			unvisited.pop();
			
			if (cur_node->_visited) continue;
			cur_node->_visited = true;

			// Just an optimization
			//if (cur_node == target->a || cur_node == target->b)
			//	break; // shortest path found (either a or b works becase shortest one will always be visited first)
			//	       // -> not quite true due to the possible difference in additional distance along the final segment! -> if either node found just check again that both are visited

			// update neighbours with new minimum distances
			for (auto& seg : cur_node->segments) {
				int dir = cur_node == seg->node_a ? 0 : 1;
				Node* other_node = dir ? seg->node_a : seg->node_b;

				int lane_i = 0;
				for (auto& lane : seg->layout->lanes) {
					if (lane.direction == dir) { // TODO: could cache list to avoid this check and iterate half as many lanes

						float new_cost = cur_node->_cost + seg->calc_length(); // TODO: cache length?
						if (new_cost < other_node->_cost) {
							other_node->_pred      = cur_node;
							other_node->_pred_seg  = seg;
							other_node->_pred_lane = lane_i;
							other_node->_cost      = new_cost;

							unvisited.push(other_node); // push updated neighbour (duplicate)
						}
					}
					lane_i++;
				}
			}
		}

		//// make path out of dijkstra graph
		if (target->node_a->_pred == nullptr && target->node_b->_pred == nullptr)
			return false; // no path found
		
		// additional distances from a and b of the target segment
		float dist_from_a = 0.5f;
		float dist_from_b = 0.5f;

		float a_cost = target->node_a->_cost + dist_from_a;
		float b_cost = target->node_b->_cost + dist_from_b;
		// choose end node that end up fastest
		Node* end_node = a_cost < b_cost ? target->node_a : target->node_b;

		assert(end_node->_cost < INF);

		std::vector<Node*> tmp_nodes;

		Node* cur = end_node;
		while (cur) {
			tmp_nodes.push_back(cur);
			cur = cur->_pred;
		}
		assert(tmp_nodes.size() >= 1);

		Node* start_node = tmp_nodes.back();

		// first node
		path->nodes.push_back(start_node);

		// start segment
		// if start node is segment.a then we go from b->a and thus need to start on a reverse lane
		int start_lane = start_node == start->node_b ? 0 : (int)start->layout->lanes.size()-1;
		path->segments.push_back({ start, start_lane });

		// nodes
		for (int i=(int)tmp_nodes.size()-2; i>=0; --i) {
			assert(tmp_nodes[i]->_pred_seg);
			// segment of predecessor to path node
			path->segments.push_back({ tmp_nodes[i]->_pred_seg, tmp_nodes[i]->_pred_lane });
			// path node
			path->nodes.push_back(tmp_nodes[i]);
		}

		// end segment
		// if end node is segment.a then we go from a->b and thus need to start on a forward lane
		int end_lane = end_node == target->node_a ? 0 : (int)target->layout->lanes.size()-1;
		path->segments.push_back({ target, end_lane });

		return true;
	}

};

struct Building {
	BuildingAsset* asset;

	float3 pos = 0;
	float  rot = 0;

	Network::Segment* connected_segment = nullptr;
};
struct Citizen {
	// TODO: needs to be some kind of state like in car, or in building
	// OR car/building etc needs to track citizen and we dont know where the citizen is
	// probably best to first use double pointers everywhere, likely that this is not a problem in terms of memory

	// TODO: i think some sort of CitizenLoc interface would work well here
	Building* building = nullptr;
	std::unique_ptr<Network::Agent> path = nullptr;

	//Building* home = nullptr;
	//Building* work = nullptr;

	CarAsset* asset;

	lrgb col;

	// TODO: get rid of this? This is not persistent data (and citizens in buildings don't need to be drawn)
	float3 _pos;
	float _rot;

	Citizen (Random& r, Building* initial_building) { // TODO: spawn citizens on map edge (on path)
		building = initial_building;

		col = hsv2rgb(r.uniformf(), 1.0f, 0.8f);
	}
};

struct Entities {

	std::vector<std::unique_ptr<Building>> buildings;
	std::vector<std::unique_ptr<Citizen>> citizens;

	bool buildings_changed = true;
};

struct Test {
	
	float2 a = float2(0,0);
	float2 b = float2(50,0);
	float2 c = float2(50,50);
	float2 d = float2(0,50);
	int count = 50;

	//float speed = 0.3f;
	float speed = 10;
	float k3 = 0;
	float k4 = 0;

	float curv_limit = 1;

	float2 prev_pos3 = 0;

	struct RollingPlot {
		float buf[1024] = {};
		int cur = 0;

		void update (float val, const char* label, float min, float max) {
			buf[cur] = val;
			ImGui::PlotLines(label, buf, ARRLEN(buf), cur, 0, min, max, ImVec2(0, 100), sizeof(buf[0]));
			
			cur = (cur+1) % ARRLEN(buf);
		}
	};
	RollingPlot plot_vel;
	RollingPlot plot_curv;
	RollingPlot plot_speed;

	void update (Renderer* renderer, Input& I) {
		if (!ImGui::TreeNodeEx("test")) return; // ImGuiTreeNodeFlags_DefaultOpen

		ImGui::DragFloat2("a", &a.x, 0.1f);
		ImGui::DragFloat2("b", &b.x, 0.1f);
		ImGui::DragFloat2("c", &c.x, 0.1f);
		ImGui::DragFloat2("d", &d.x, 0.1f);
		ImGui::DragInt("count", &count, 0.1f);

		ImGui::SliderAngle("curv_limit", &curv_limit, 0, 180);

		ImGui::DragFloat("speed", &speed, 0.1f);
		ImGui::SliderFloat("k3", &k3, 0, 1);
		ImGui::SliderFloat("k4", &k4, 0, 1);

		float2 prev3 = a;
		float2 prev4 = a;

		// numerical length calc seemingly always within a few % (<1% often) even with just 10 segments!
		// I don't think feasable analytical solutions exist, at least for bezier4
		// bezier3 contained wierd trig functions already
		// -> just cache the length for roads & intersections?
		float len3 = 0;
		float len4 = 0;

		for (int i=0; i<count; ++i) {
			float t = (float)(i+1) / (float)count;
			
			auto val3 = bezier3(t, a,b,d);
			auto val4 = bezier4(t, a,b,c,d);

			renderer->dbgdraw.line(float3(prev3,0), float3(val3.pos,0), lrgba(1,0,0,1));
			renderer->dbgdraw.line(float3(prev4,0), float3(val4.pos,0), lrgba(1,0,1,1));

			len3 += distance(prev3, val3.pos);
			len4 += distance(prev4, val4.pos);

			prev3 = val3.pos;
			prev4 = val4.pos;
		}

		renderer->dbgdraw.wire_circle(float3(a,0), 1, lrgba(1,1,0,1));
		renderer->dbgdraw.wire_circle(float3(b,0), 1, lrgba(1,0.7f,0.3f,1));
		renderer->dbgdraw.wire_circle(float3(c,0), 1, lrgba(1,0.7f,0.3f,1));
		renderer->dbgdraw.wire_circle(float3(d,0), 1, lrgba(1,1,0,1));

		ImGui::Text("len3: %.2f", len3);
		ImGui::Text("len4: %.2f", len4);

		{
			auto val3 = bezier3(k3, a,b,d);
			auto val4 = bezier4(k4, a,b,c,d);

			// normalized 'velocity' per bezier k
			//float2 approx_vel3 = (val3.pos - prev_pos3) / (I.dt * speed);

			auto draw = [&] (BezierRes& val, lrgba col, float& k) {
				renderer->dbgdraw.wire_circle(float3(val.pos,0), 1, col);
				
				renderer->dbgdraw.vector(float3(val.pos,0), float3(val.vel,0) * 0.1f, col);
				//renderer->dbgdraw.vector(float3(val3.pos,0), float3(val3.accel,0) * 0.1f, lrgba(1,0,1,1));
				renderer->dbgdraw.vector(float3(val.pos,0), float3(0,0,abs(val.curv))*20, col);

				float turn_radius = 1.0f / val.curv;
				float2 dir = normalizesafe(val.vel);
				float2 circ_pos = val.pos + rotate90(dir) * turn_radius;
				renderer->dbgdraw.wire_circle(float3(circ_pos,0), turn_radius, col, 64);

				float capped_speed = speed;
				float ang_per_sec = abs(val.curv) * speed; // ang/dist (curv) * dist/sec (speed) -> ang/sec
				if (ang_per_sec > curv_limit) // curv_limit (ang/sec) / val.curv (ang/dist) = speed (dist/sec)
					capped_speed = curv_limit / abs(val.curv);

				//float curv_max_speed = curv_limit / val.curv;
				//float capped_speed = min(curv_max_speed, speed);

				float speed_over_k = length(val.vel);
				k += (capped_speed * I.dt) / speed_over_k;
				//k += speed * I.dt;
				k = fmodf(k, 1.0f);

				plot_vel.update(length(val.vel), "bezier speed", 0, 200);
				plot_curv.update(val.curv, "curv", -0.2f, 0.2f);
				plot_speed.update(capped_speed, "real speed", 0, 30);
			};
			draw(val3, lrgba(0,1,0,1), k3);
			//draw(val4, lrgba(0,1,1,1), k4);

			//prev_pos3 = val3.pos;
		}

		ImGui::TreePop();
	}
};

struct App : public Engine {

	App (): Engine{"Kiss-Framework Project"} {}
	virtual ~App () {}
	
	friend SERIALIZE_TO_JSON(App)   { SERIALIZE_TO_JSON_EXPAND(cam, assets); }
    friend SERIALIZE_FROM_JSON(App) {
		SERIALIZE_FROM_JSON_EXPAND(cam, assets);

	}

	virtual void json_load () { load("debug.json", this); }
	virtual void json_save () { save("debug.json", *this); }
	
	Assets assets;
	Entities entities;
	Network net;

	float sun_azim = deg(30); // degrees from east, counter clockwise
	float sun_elev = deg(14);
	float day_t = 0.6f; // [0,1] -> [0,24] hours
	float day_speed = 1.0f / 60.0f;
	bool  day_pause = true;

	GameCamera cam = GameCamera{ float3(8,8,0)*1024 };
	View3D view;

	float3 sun_dir;

	std::unique_ptr<Renderer> renderer = create_ogl_backend();

	void spawn () {
		static int buildings_n = 10;
		static int citizens_n = 200;

		bool buildings = ImGui::SliderInt("buildings_n", &buildings_n, 1, 1000) || assets.assets_reloaded;
		bool citizens  = ImGui::SliderInt("citizens_n", &citizens_n, 1, 1000) || entities.buildings_changed;

		if (buildings || citizens) {
			ZoneScopedN("spawn buildings");

			entities.buildings.clear();
			net = {};

			Random rand(0);

			auto base_pos = float3(8,8,0)*1024;
			auto* house0 = assets.buildings[0].get();
			auto* house1 = assets.buildings[1].get();

			net.nodes.resize((buildings_n+1)*(buildings_n+1));
			
			auto get_node = [&] (int x, int y) -> Network::Node* {
				return net.nodes[y * (buildings_n+1) + x].get();
			};
			
			auto* road_layout = assets.road_layouts[0].get();

			// create path nodes grid
			for (int y=0; y<buildings_n+1; ++y)
			for (int x=0; x<buildings_n+1; ++x) {
				float3 pos = base_pos + float3((float)x,(float)y,0) * (float3(40,25,0) + float3(road_layout->width, road_layout->width, 0));
				net.nodes[y * (buildings_n+1) + x] = std::make_unique<Network::Node>(Network::Node{pos});
			}
			
			auto create_segment = [&] (Network::Node* a, Network::Node* b) {
				assert(a && b && a != b);

				auto* seg = net.segments.emplace_back(std::make_unique<Network::Segment>(Network::Segment{
					road_layout, a, b
				})).get();

				a->segments.push_back(seg);
				b->segments.push_back(seg);

				seg->agents.lanes.resize(road_layout->lanes.size());
			};

			// create x paths
			for (int y=0; y<buildings_n+1; ++y)
			for (int x=0; x<buildings_n; ++x) {
				auto* a = get_node(x, y);
				auto* b = get_node(x+1, y);
				create_segment(a, b);
			}
			// create y paths
			for (int y=0; y<buildings_n; ++y)
			for (int x=0; x<buildings_n+1; ++x) {
				if (rand.chance(0.5f)) {
					auto* a = get_node(x, y);
					auto* b = get_node(x, y+1);
					create_segment(a, b);
				}
			}
			
			for (int y=0; y<buildings_n; ++y)
			for (int x=0; x<buildings_n; ++x) {
				Random rand(hash(int2(x,y))); // position-based rand
				auto* asset = rand.uniformi(0, 2) ? house0 : house1;

				float3 pos = base_pos + (float3((float)x,(float)y,0) + float3(0.5f)) * (float3(40,25,0) + float3(road_layout->width, road_layout->width, 0));
				auto& build = entities.buildings.emplace_back(std::make_unique<Building>(Building{ asset, pos }));

				auto* a = get_node(x, y);
				auto* b = get_node(x+1, y);
				for (auto& seg : a->segments) {
					auto* other_node = seg->node_a != a ? seg->node_a : seg->node_b;
					if (other_node == b) {
						// found path in front of building
						build->connected_segment = seg;
						break;
					}
				}

				assert(build->connected_segment);
			}

			entities.buildings_changed = true;
		}
		
		if (citizens) {
			ZoneScopedN("spawn citizens");

			auto* car_asset = assets.cars[0].get();

			entities.citizens.clear();
			entities.citizens.resize(citizens_n);

			if (entities.buildings.size() > 0) {
				for (int i=0; i<citizens_n; ++i) {
					auto* building = entities.buildings[random.uniformi(0, (int)entities.buildings.size())].get();
					entities.citizens[i] = std::move( std::make_unique<Citizen>(Citizen{random, building}) );
					entities.citizens[i]->asset = car_asset;
				}
			}
		}
	}

	virtual void imgui () {
		ZoneScoped;

		renderer->imgui(*this);

		ImGui::Separator();

		cam.imgui("cam");

		if (ImGui::TreeNode("Time of Day")) {

			ImGui::SliderAngle("sun_azim", &sun_azim, 0, 360);
			ImGui::SliderAngle("sun_elev", &sun_elev, -90, 90);
			ImGui::SliderFloat("day_t", &day_t, 0,1);

			ImGui::SliderFloat("day_speed", &day_speed, 0, 0.25f, "%.3f", ImGuiSliderFlags_Logarithmic);
			ImGui::Checkbox("day_pause", &day_pause);
			
			ImGui::TreePop();
		}
	}

	int pathing_count = 0;

	void update_citizens () {
		ZoneScoped;

		static float speed = 30;
		ImGui::SliderFloat("speed", &speed, 0, 500);
		
		static int dbg_segment = 135;
		ImGui::DragInt("dbg_segment", &dbg_segment, 0.1f);

		auto update_agent = [&] (Network::Agent* agent) {
			struct Move {
				// bezier points
				float3 a, b, c;
				Network::Agents* agents = nullptr;
				int lane = -1;
			};
			auto count = [&] () {
				assert(agent->nodes.size() + 1 == agent->segments.size());
				return (int)agent->nodes.size()*2 + 1 + 2;
			};
			auto get_move = [&] (int idx) -> Move {
				// Ughhh. Generator expressions would be sooo nice

				auto s_seg = agent->segments.front();
				auto e_seg = agent->segments.back();

				auto s_lane = s_seg.seg->clac_lane_info(s_seg.lane);
				auto e_lane = e_seg.seg->clac_lane_info(e_seg.lane);

				float3 s0 = agent->start->pos;
				float3 s1 = (s_lane.a + s_lane.b) * 0.5f;
				float3 e0 = (e_lane.a + e_lane.b) * 0.5f;
				float3 e1 = agent->end->pos;

				int count = (int)agent->nodes.size()*2 + 1;

				if (idx == 0) {
					return { s0, (s0+s1)*0.5f, s1 };
				}
				idx -= 1; // ingore first

				if (idx < count) {
					auto& seg = agent->segments[idx/2];
					auto l = seg.seg->clac_lane_info(seg.lane);

					if (idx % 2 == 0) { // segment
						if (idx == 0)            l.a = s1;
						else if (idx == count-1) l.b = e0;
						return { l.a, (l.a+l.b)*0.5f, l.b, &seg.seg->agents, seg.lane };
					}
					else { // node
						auto& seg2 = agent->segments[idx/2+1];
						auto l2 = seg2.seg->clac_lane_info(seg2.lane);

						float2 point;
						if (!line_line_intersect((float2)l.a, (float2)l.b, (float2)l2.a, (float2)l2.b, &point))
							point = (l.b+l2.a)*0.5f;

						Move m = { l.b, float3(point, l.a.z), l2.a };
						return m;
					}
				}
					
				return { e0, (e0+e1)*0.5f, e1 };
			};
			auto query_dist_to_next = [] (Network::Agents* agents, Network::Agent* agent, int lane) {
				float min_dist = INF;
				if (agents) {
					for (auto& a : agents->lanes[lane]) {
						if (a->cur_t <= agent->cur_t) {
							// agent behind, can ignore
							continue;
						}
						float dist = distance(a->cit->_pos, agent->cit->_pos);
						if (dist < min_dist) {
							min_dist = dist;
						}
					}
				}
				return min_dist;
			};

			assert(agent->nodes.size() >= 1);
			assert(agent->idx < count());
			assert(agent->cur_t < 1.0f);

			auto move = get_move(agent->idx);

			auto bez = bezier3(agent->cur_t, (float2)move.a, (float2)move.b, (float2)move.c);
			float bez_speed = length(bez.vel); // delta pos / bezier t

			float dist_to_next = query_dist_to_next(move.agents, agent, move.lane);
			float slowdown = clamp(map(dist_to_next, 2.0f, 8.0f), 0.0f, 1.0f);
			
			agent->cit->_pos = float3(bez.pos, move.a.z);
			agent->cit->_rot = bez_speed > 0 ? atan2f(-bez.vel.x, bez.vel.y) : 0; // atan with roated axes since rot=0 should be +y in my convention

			if (agent->cit == entities.citizens[0].get()) {

				// draw target building
				float2 size = (float2)agent->end->asset->size;
				renderer->dbgdraw.wire_quad(float3(agent->end->pos - float3(size*0.5f,0)), size, lrgba(0,1,0,1));

				auto m = get_move(agent->idx);
				renderer->dbgdraw.line(agent->cit->_pos + float3(0,0,1), m.c + float3(0,0,1), lrgba(0,1,0,1));
				for (int i=agent->idx+1; i<count(); ++i) {
					auto m = get_move(i);
					renderer->dbgdraw.line(m.a + float3(0,0,1), m.c + float3(0,0,1), lrgba(0,1,0,1));
				}
			}

			// (delta pos / delta time)[speed] * time[dt] / (delta pos / bezier delta t)[bez_speed]
			// -> delta t
			agent->cur_t += min(speed * input.dt * slowdown, dist_to_next) / bez_speed;

			if (agent->cur_t >= 1.0f) {
				agent->idx++;
				agent->cur_t = 0;

				if (move.agents) move.agents->remove(agent, move.lane); // exit current lane

				if (agent->idx >= count()) {
					// end of path reached
					agent->cit->building = agent->end;
					agent->cit->path = nullptr;
					return;
				}

				move = get_move(agent->idx);
				if (move.agents) move.agents->add(agent, move.lane); // enter next lane
			}
		};

		for (auto& cit : entities.citizens) {
			if (cit->building) {
				auto* cur_target = entities.buildings[ random.uniformi(0, (int)entities.buildings.size()) ].get();
				
				cit->_pos = cit->building->pos;
				cit->_rot = 0;

				assert(cit->building->connected_segment);
				if (cit->building->connected_segment) {
					ZoneScopedN("pathfind");

					auto path = std::make_unique<Network::Agent>();
					path->cit = cit.get();
					path->start = cit->building;
					path->end   = cur_target;
					bool valid = net.pathfind(cit->building->connected_segment, cur_target->connected_segment, path.get());

					pathing_count++;

					if (valid) {
						cit->building = nullptr;
						cit->path = std::move(path);
					}
				}
			}
			else {
				update_agent(cit->path.get());
			}
			
			//float rad = cit == entities.citizens[0] ? 5.0f : 2.0f;
			//renderer->dbgdraw.cylinder(cur_pos, rad, 1.7f, lrgba(cit->col, 1), 8);
		}

		if (dbg_segment >= 0 && dbg_segment < (int)net.segments.size()) {
			auto& seg = net.segments[dbg_segment];
			
			renderer->dbgdraw.line(seg->node_a->pos, seg->node_b->pos, lrgba(1,1,0,1));

			for (auto& lane : seg->agents.lanes) {
				for (auto& agent : lane) {
					renderer->dbgdraw.wire_circle(agent->cit->_pos, 2, lrgba(1,1,0,1));
				}
			}
		}
	}

	Test test;

	void update () {
		ZoneScoped;

		test.update(renderer.get(), input);

		spawn();

		if (!day_pause) day_t = wrap(day_t + day_speed * input.dt, 1.0f);
		sun_dir = rotate3_Z(sun_azim) * rotate3_X(sun_elev) * rotate3_Y(day_t * deg(360)) * float3(0,0,-1);

		view = cam.update(input, (float2)input.window_size);

		renderer->dbgdraw.axis_gizmo(view, input.window_size);


		//for (auto& node : net.nodes) {
		//	renderer->dbgdraw.wire_cube(node->pos - 5*0.5f, 5, lrgba(1,1,0,1));
		//}
		//for (auto& seg : net.segments) {
		//	renderer->dbgdraw.line(seg->a->pos, seg->b->pos, lrgba(1,1,0,1));
		//}

		pathing_count = 0;

		for (auto& building : entities.buildings) {
			//renderer->dbgdraw.quad(float3(building->pos), (float2)building->asset->size, lrgba(1,1,1,1));
			//renderer->dbgdraw.wire_quad(float3(building->pos), (float2)building->asset->size, lrgba(1,1,1,1));
		}

		update_citizens();

		static RunningAverage pathings_avg(30);
		pathings_avg.push((float)pathing_count);
		float min, max;
		float avg = pathings_avg.calc_avg(&min, &max);
		ImGui::Text("pathing_count: avg %3.1f min: %3.1f max: %3.1f", avg, min, max);
	}

	virtual void frame () {
		ZoneScoped;
		
		renderer->begin(*this);
		renderer->dbgdraw.clear();

		update();

		renderer->end(*this);


		assets.assets_reloaded = false;
		entities.buildings_changed = false;
	}
};
