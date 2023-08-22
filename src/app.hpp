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
	// TODO: naming
	// path should be reserved for pathfinding
	// general name to call a network edges? (road, pedestrian path, rail track etc.)
	// how to call intersections? node seems fine

	struct Segment;

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

	struct ActivePath {
		struct Seg {
			Segment* seg;
			int      lane;
		};

		int   idx = 0;
		float cur_t = 0;
		std::vector<Node*> nodes;
		std::vector<Seg>   segments;

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
				
				struct Move {
					float3 a, b;
				};
				auto count = [&] () {
					assert(path.nodes.size() + 1 == path.segments.size());
					return (int)path.nodes.size()*2 + 1 + 2;
				};
				auto get_move = [&] (int idx) -> Move {
					// Ughhh. Generator expressions would be sooo nice

					auto s_seg = path.segments.front();
					auto e_seg = path.segments.back();

					auto s_lane = s_seg.seg->clac_lane_info(s_seg.lane);
					auto e_lane = e_seg.seg->clac_lane_info(e_seg.lane);

					float3 s0 = path.start->pos;
					float3 s1 = (s_lane.a + s_lane.b) * 0.5f;
					float3 e0 = (e_lane.a + e_lane.b) * 0.5f;
					float3 e1 = path.end->pos;

					int count = (int)path.nodes.size()*2 + 1;

					if (idx == 0) {
						return { s0, s1 };
					}
					idx -= 1; // ingore first

					if (idx < count) {
						if (idx % 2 == 0) { // segment

							auto& seg = path.segments[idx/2];
							auto lane_info = seg.seg->clac_lane_info(seg.lane);

							Move m = { lane_info.a, lane_info.b };
							if (idx == 0)            m.a = s1;
							else if (idx == count-1) m.b = e0;
							return m;
						}
						else { // node
							
							auto& sa = path.segments[idx/2];
							auto& sb = path.segments[idx/2+1];
							auto la = sa.seg->clac_lane_info(sa.lane);
							auto lb = sb.seg->clac_lane_info(sb.lane);

							Move m = { la.b, lb.a };
							return m;
						}
					}
					
					return { e0, e1 };
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

			float rad = cit == entities.citizens[0] ? 5.0f : 2.0f;
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
