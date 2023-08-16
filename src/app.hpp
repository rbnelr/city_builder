#pragma once
#include "common.hpp"
#include "engine/engine.hpp"
#include "game_camera.hpp"
#include "assets.hpp"

#include <queue>

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

	struct Segment;

	struct Node {
		float3 pos;
		std::vector<Segment*> segments;

		// for Dijkstra, TODO: remove this from this data structure! indices instead of pointers needed to be able to have seperate node lists?
		// else always need to map pointers to other pointers or indices
		float _dist;
		bool  _visited;
		Node* _pred;
	};

	struct Segment { // better name? Keep Path and call path Route?
		// SegmentLayout* -> type of lanes etc.
		Node* a;
		Node* b;
		float length;
		
		void calc_length () {
			length = distance(a->pos, b->pos);
		}
	};

	std::vector<std::unique_ptr<Node>> nodes;
	std::vector<std::unique_ptr<Segment>> segments;

	struct ActivePath {
		float length = 0;
		int   idx = 0;
		float cur_t = 0;
		std::vector<Node*> nodes;

		Building* start = nullptr;
		Building* end   = nullptr;
	};

	bool pathfind (Segment* start, Segment* target, ActivePath* path) {
		// use dijstra

		struct Comparer {
			bool operator () (Node* l, Node* r) {
				// true: l < r
				// but since priority_queue only lets us get
				// max element rather than min flip everything around
				return l->_dist > r->_dist;
			}
		};
		std::priority_queue<Node*, std::vector<Node*>, Comparer> unvisited;
		
		// queue all nodes
		for (auto& node : nodes) {
			node->_dist = INF;
			node->_visited = false;
			node->_pred = nullptr;
		}

		// handle the two start nodes
		start->a->_dist = start->length / 0.5f; // pretend start point is at center of start segment for now
		start->b->_dist = start->length / 0.5f;
		
		unvisited.push(start->a);
		unvisited.push(start->b);

		while (!unvisited.empty()) {
			// visit node with min distance
			Node* cur = unvisited.top();
			unvisited.pop();
			
			if (cur->_visited) continue;
			cur->_visited = true;

			if (cur == target->a || cur == target->b)
				break; // shortest path found (either a or b works becase shortest one will always be visited first)

			// update neighbours with new minimum distances
			for (auto& seg : cur->segments) {
				Node* neighbour = seg->a != cur ? seg->a : seg->b;

				float new_dist = cur->_dist + seg->length;
				if (new_dist < neighbour->_dist) {
					neighbour->_pred = cur;
					neighbour->_dist = new_dist;

					unvisited.push(neighbour); // push updated neighbour (duplicate)
				}
			}
		}

		//// make path out of dijkstra graph
		if (target->a->_pred == nullptr && target->b->_pred == nullptr)
			return false; // no path found
		
		// additional distances from a and b of the target segment
		float dist_from_a = 0.5f;
		float dist_from_b = 0.5f;

		float a_len = target->a->_dist + dist_from_a;
		float b_len = target->b->_dist + dist_from_b;
		// choose end node that end up fastest
		Node* end_node = a_len < b_len ? target->a : target->b;

		path->length = end_node->_dist;
		assert(path->length < INF);

		std::vector<Node*> tmp_nodes;

		Node* cur = end_node;
		while (cur) {
			tmp_nodes.push_back(cur);
			cur = cur->_pred;
		}

		for (auto it=tmp_nodes.rbegin(); it!=tmp_nodes.rend(); ++it) {
			path->nodes.push_back(*it);
		}

		//path->nodes.push_back(target->a == end_node ? target->b : target->a);

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

	// TODO: i think some sort of CitizenLoc interface would work well here (implement in ActivePath 
	Building* building = nullptr;
	std::unique_ptr<Network::ActivePath> path = nullptr;

	//Building* home = nullptr;
	//Building* work = nullptr;

	lrgb col;

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

struct App : public Engine {

	App (): Engine{"Kiss-Framework Project"} {}
	virtual ~App () {}
	
	friend SERIALIZE_TO_JSON(App)   { SERIALIZE_TO_JSON_EXPAND(cam, assets); }
    friend SERIALIZE_FROM_JSON(App) {
		t.assets = Assets();
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

	const float3 street_size = float3(12, 12, 1);

	void spawn () {
		static int buildings_n = 10;
		static int citizens_n = 50;

		if (ImGui::SliderInt("buildings_n", &buildings_n, 1, 1000) || assets.assets_reloaded) {
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
			
			// create path nodes grid
			for (int y=0; y<buildings_n+1; ++y)
			for (int x=0; x<buildings_n+1; ++x) {
				float3 pos = base_pos + float3((float)x,(float)y,0) * (float3(40,25,0) + street_size);
				net.nodes[y * (buildings_n+1) + x] = std::make_unique<Network::Node>(Network::Node{pos});
			}
			
			auto create_segment = [&] (Network::Node* a, Network::Node* b) {
				assert(a && b && a != b);

				auto* seg = net.segments.emplace_back(std::make_unique<Network::Segment>(Network::Segment{})).get();
				seg->a = a;
				seg->b = b;

				a->segments.push_back(seg);
				b->segments.push_back(seg);
				
				seg->calc_length();
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

				float3 pos = base_pos + (float3((float)x,(float)y,0) + float3(0.5f)) * (float3(40,25,0) + street_size);
				auto& build = entities.buildings.emplace_back(std::make_unique<Building>(Building{ asset, pos }));

				auto* a = get_node(x, y);
				auto* b = get_node(x+1, y);
				for (auto& seg : a->segments) {
					auto* other_node = seg->a != a ? seg->a : seg->b;
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
		
		if (ImGui::SliderInt("citizens_n", &citizens_n, 1, 1000) || entities.buildings_changed) {
			ZoneScopedN("spawn citizens");

			if (entities.buildings_changed)
				entities.citizens.clear(); // invalidated pointers

			int old_size = (int)entities.citizens.size();
			entities.citizens.resize(citizens_n);

			if (entities.buildings.size() > 0) {
				for (int i=old_size; i<citizens_n; ++i) {
					auto* building = entities.buildings[random.uniformi(0, (int)entities.buildings.size())].get();
					entities.citizens[i] = std::move( std::make_unique<Citizen>(Citizen{random, building}) );
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

		static float speed = 100;
		ImGui::SliderFloat("speed", &speed, 0, 500);

		for (auto& cit : entities.citizens) {
			if (cit->building) {
				auto* cur_target = entities.buildings[ random.uniformi(0, (int)entities.buildings.size()) ].get();

				assert(cit->building->connected_segment);
				if (cit->building->connected_segment) {
					ZoneScopedN("pathfind");

					auto path = std::make_unique<Network::ActivePath>();
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

			float3 cur_pos = 0;

			if (cit->path) {
				auto& path = *cit->path;

				auto count = [&] () {
					return (int)path.nodes.size()-1 + 4;
				};
				struct Move {
					float3 a, b;
				};
				auto get_move = [&] (int idx) -> Move {
					// Ughhh. Generator expressions would be sooo nice
					float3 s0 = path.start->pos;
					float3 s1 = (path.start->connected_segment->a->pos + path.start->connected_segment->b->pos) * 0.5f;
					float3 e0 = (path.end->connected_segment->a->pos + path.end->connected_segment->b->pos) * 0.5f;
					float3 e1 = path.end->pos;

					if (idx == 0) {
						return { s0, s1 };
					}
					if (idx == 1) {
						return { s1, path.nodes.front()->pos };
					}

					if (idx-2 < (int)path.nodes.size()-1) {
						return { path.nodes[idx-2]->pos, path.nodes[idx-2 + 1]->pos };
					}

					if (idx == (int)path.nodes.size()-1 + 2) {
						return { path.nodes.back()->pos, e0 };
					}
					else /*idx == (int)path.nodes.size()-1 + 3*/ {
						return { e0, e1 };
					}
				};

				assert(path.nodes.size() >= 1);
				assert(path.idx < count());
				assert(path.cur_t < 1.0f);

				auto move = get_move(path.idx);

				float len = distance(move.a, move.b);

				path.cur_t += (speed * input.dt) / len;

				if (path.cur_t >= 1.0f) {
					path.idx++;
					path.cur_t = 0;

					move = get_move(path.idx);
				}

				if (path.idx >= count()) {
					// end of path reached
					cit->building = path.end;
					cit->path = nullptr;
				}
				else {
					cur_pos = lerp(move.a, move.b, path.cur_t);

					if (cit == entities.citizens[0]) {

						// draw target building
						float2 size = (float2)path.end->asset->size;
						renderer->dbgdraw.wire_quad(float3(path.end->pos - float3(size*0.5f,0)), size, lrgba(0,1,0,1));

						auto m = get_move(path.idx);
						renderer->dbgdraw.line(cur_pos + float3(0,0,1), m.b + float3(0,0,1), lrgba(0,1,0,1));
						for (int i=path.idx+1; i<count(); ++i) {
							auto m = get_move(i);
							renderer->dbgdraw.line(m.a + float3(0,0,1), m.b + float3(0,0,1), lrgba(0,1,0,1));
						}
					}
				}
			}

			if (cit->building) {
				cur_pos = cit->building->pos;
			}

			float rad = cit == entities.citizens[0] ? 5.0f : 3.0f;
			renderer->dbgdraw.cylinder(cur_pos, rad, 1.7f, lrgba(cit->col, 1), 8);
		}
	}

	void update () {
		ZoneScoped;

		spawn();

		if (!day_pause) day_t = wrap(day_t + day_speed * input.dt, 1.0f);
		sun_dir = rotate3_Z(sun_azim) * rotate3_X(sun_elev) * rotate3_Y(day_t * deg(360)) * float3(0,0,-1);

		renderer->dbgdraw.clear();

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

		update();

		renderer->end(*this);


		assets.assets_reloaded = false;
		entities.buildings_changed = false;
	}
};
