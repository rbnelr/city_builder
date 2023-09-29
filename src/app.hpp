#pragma once
#include "common.hpp"
#include "engine/engine.hpp"
#include "game_camera.hpp"
#include "assets.hpp"
#include "util.hpp"
#include "network.hpp"

struct App;

struct Renderer {

	virtual ~Renderer () {}
	
	virtual void imgui (App& app) = 0;

	virtual void begin (App& app) = 0;
	virtual void end (App& app) = 0;
};

std::unique_ptr<Renderer> create_ogl_backend ();


inline int length2int (float len) {
	return ceili(len * 100.0f);
}
inline constexpr float LANE_COLLISION_R = 1.3f;
inline constexpr float CAR_SIZE = 3.5f;


struct Citizen;
struct Building;

typedef NullableVariant<Citizen*, network::Node*> sel_ptr;


struct Building {
	BuildingAsset* asset;

	float3 pos = 0;
	float  rot = 0;

	network::Segment* connected_segment = nullptr;
};

// Acts more like a Vehicle currently
struct Citizen {
	// TODO: needs to be some kind of state like in car, or in building
	// OR car/building etc needs to track citizen and we dont know where the citizen is
	// probably best to first use double pointers everywhere, likely that this is not a problem in terms of memory

	// TODO: i think some sort of CitizenLoc interface would work well here
	Building* building = nullptr;
	std::unique_ptr<network::Agent> path = nullptr;

	//Building* home = nullptr;
	//Building* work = nullptr;

	CarAsset* asset;

	lrgb col;

	// TODO: get rid of this? This is not persistent data (and citizens in buildings don't need to be drawn)
	float3 front_pos;
	float3 rear_pos;

	float3 center () { return (front_pos + rear_pos)*0.5; };

	Citizen (Random& r, Building* initial_building) { // TODO: spawn citizens on map edge (on path)
		building = initial_building;

		col = hsv2rgb(r.uniformf(), 1.0f, 0.8f);
	}

	SelCircle get_sel_shape () {
		return { center(), CAR_SIZE*0.5f, lrgb(0.04f, 1, 0.04f) };
	}
};

struct Entities {

	std::vector<std::unique_ptr<Building>> buildings;
	std::vector<std::unique_ptr<Citizen>> citizens;

	bool buildings_changed = true;
};

struct Test {
	//float2 a = float2(0, 0);
	//float2 b = float2(50, 50);
	//float2 c = float2(0, 50);
	//float2 d = float2(50, 0);
	//float2 e = float2(50, 0);
	//float2 f = float2(50, 0);
	//
	//float r = 10;

	void update (Input& I, View3D& view) {
		//ImGui::DragFloat2("a", &a.x, 1);
		//ImGui::DragFloat2("b", &b.x, 1);
		//ImGui::DragFloat2("c", &c.x, 1);
		//ImGui::DragFloat2("d", &d.x, 1);
		//ImGui::DragFloat2("e", &e.x, 1);
		//ImGui::DragFloat2("f", &f.x, 1);
		//
		//draggable(I, view, &a, r);
		//draggable(I, view, &b, r);
		//draggable(I, view, &c, r);
		//draggable(I, view, &d, r);
		//draggable(I, view, &e, r);
		//draggable(I, view, &f, r);
		//
		//g_dbgdraw.point(float3(a,0), 5, lrgba(1,0,0,1));
		//g_dbgdraw.point(float3(b,0), 5, lrgba(1,1,0,1));
		//g_dbgdraw.point(float3(c,0), 5, lrgba(0,1,0,1));
		//g_dbgdraw.point(float3(d,0), 5, lrgba(0,0,1,1));
		//g_dbgdraw.point(float3(e,0), 5, lrgba(0,0,1,1));
		//g_dbgdraw.point(float3(f,0), 5, lrgba(0,0,1,1));
		//
		//float u;
		//if (!ray_box_intersection(a, b-a, c, d-c, length(d-c), r*2, &u))
		//	return;
		//
		//float2 j = a + (b-a)*u;
		//g_dbgdraw.point(float3(j,0), 2.5f, lrgba(0,1,1,1));
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

	virtual void imgui () {
		ZoneScoped;

		renderer->imgui(*this);

		ImGui::Separator();

		cam.imgui("cam");
		dbg_cam.imgui("dbg_cam");
		ImGui::SameLine();
		ImGui::Checkbox("View", &view_dbg_cam);

		if (ImGui::TreeNode("Time of Day")) {

			ImGui::SliderAngle("sun_azim", &sun_azim, 0, 360);
			ImGui::SliderAngle("sun_elev", &sun_elev, -90, 90);
			ImGui::SliderFloat("day_t", &day_t, 0,1);

			ImGui::SliderFloat("day_speed", &day_speed, 0, 0.25f, "%.3f", ImGuiSliderFlags_Logarithmic);
			ImGui::Checkbox("day_pause", &day_pause);
			
			ImGui::TreePop();
		}

		ImGui::Checkbox("sim_paused", &sim_paused);

		net.imgui();
	}

	
	Assets assets;
	Entities entities;
	network::Network net;

	float sun_azim = deg(30); // degrees from east, counter clockwise
	float sun_elev = deg(14);
	float day_t = 0.6f; // [0,1] -> [0,24] hours
	float day_speed = 1.0f / 60.0f;
	bool  day_pause = true;

	GameCamera cam = GameCamera{ float3(8,8,0) * 1024 };
	View3D view;

	Flycam dbg_cam = Flycam(0, 0, 100);
	bool view_dbg_cam = false;
	bool dbg_cam_cursor_was_enabled;

	float3 sun_dir;

	std::unique_ptr<Renderer> renderer = create_ogl_backend();

	bool sim_paused = false;

	Random test_rand;

	Test test;

	sel_ptr selection;
	sel_ptr selection2;

	template <typename T>
	void clear_sel () {
		if (selection.get<T>())
			selection = nullptr;
		if (selection2.get<T>())
			selection2 = nullptr;
	}

	void update_selection () {
		Ray ray;
		if (!view.cursor_ray(input, &ray.pos, &ray.dir))
			return;

		sel_ptr hover;
		float dist = INF;

		for (auto& cit : entities.citizens) {
			auto shape = cit->get_sel_shape();
			float hit_dist;
			if (shape.test(ray, &hit_dist) && hit_dist < dist) {
				hover = cit.get();
				dist = hit_dist;
			}
		}
		for (auto& node : net.nodes) {
			auto shape = node->get_sel_shape();
			float hit_dist;
			if (shape.test(ray, &hit_dist) && hit_dist < dist) {
				hover = node.get();
				dist = hit_dist;
			}
		}
		
		if (input.buttons[MOUSE_BUTTON_LEFT].went_down) {
			auto& s = input.buttons[KEY_LEFT_CONTROL].is_down ? selection2 : selection;
			s = hover;
		}
		if (hover) {
			hover.visit([&] (auto& x) {
				x->get_sel_shape().highlight();
			});
		}
		if (selection) {
			selection.visit([&] (auto& x) {
				x->get_sel_shape().highlight_selected();
			});
		}
		if (selection2) {
			selection2.visit([&] (auto& x) {
				x->get_sel_shape().highlight_selected(0.3f, lrgba(1,0,0, 1));
			});
		}
	}

	void spawn () {
		using namespace network;

		static int buildings_n = 10;
		static int citizens_n = 600;

		static float intersection_scale = 1;

		bool buildings = ImGui::SliderInt("buildings_n", &buildings_n, 1, 1000)
			|| assets.assets_reloaded;
		buildings = ImGui::Button("Respawn buildings") || buildings;

		bool citizens  = ImGui::SliderInt("citizens_n",  &citizens_n,  1, 1000)
			|| assets.assets_reloaded || buildings;
		citizens = ImGui::Button("Respawn Citizens") || citizens;

		ImGui::SliderFloat("intersection_scale", &intersection_scale, 0.1f, 4);

		if (buildings) {
			ZoneScopedN("spawn buildings");

			clear_sel<Node*>();
			//clear_sel<Segment*>();

			entities.buildings.clear();
			net = {};

			Random rand(0);

			auto base_pos = float3(100,100,0);
			auto* house0 = assets.buildings[0].get();
			auto* house1 = assets.buildings[1].get();

			net.nodes.resize((buildings_n+1)*(buildings_n+1));
			
			auto get_node = [&] (int x, int y) -> Node* {
				return net.nodes[y * (buildings_n+1) + x].get();
			};
			
			auto* road_layout = assets.road_layouts[0].get();
			
			float node_r = road_layout->width*0.5f * intersection_scale;
			float2 spacing = float2(50, 50) + node_r;

			// create path nodes grid
			for (int y=0; y<buildings_n+1; ++y)
			for (int x=0; x<buildings_n+1; ++x) {
				float3 pos = base_pos + float3((float)x,(float)y,0) * float3(spacing, 0);
				net.nodes[y * (buildings_n+1) + x] = std::make_unique<Node>(Node{pos, node_r});
			}
			
			auto create_segment = [&] (Node* a, Node* b) {
				assert(a && b && a != b);

				auto* seg = net.segments.emplace_back(std::make_unique<Segment>(Segment{
					road_layout, a, b
				})).get();

				a->segments.push_back(seg);
				b->segments.push_back(seg);

				seg->agents.lanes.resize(road_layout->lanes.size());

				seg->update_cached();
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

			for (auto& node : net.nodes) {
				node->update_cached(); // update seg connections
			}

			for (int y=0; y<buildings_n; ++y)
			for (int x=0; x<buildings_n; ++x) {
				Random rand(hash(int2(x,y))); // position-based rand
				auto* asset = rand.uniformi(0, 2) ? house0 : house1;

				float3 pos = base_pos + (float3((float)x,(float)y,0) + float3(0.5f)) * float3(spacing, 0);
				float rot = deg(90);
				auto& build = entities.buildings.emplace_back(std::make_unique<Building>(Building{ asset, pos, rot }));

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

			clear_sel<Citizen*>();

			// remove references
			for (auto& node : net.nodes) {
				node->agents.free.list.clear();
			}
			for (auto& seg : net.segments) {
				for (auto& lane : seg->agents.lanes) {
					lane.list.clear();
				}
			}

			entities.citizens.clear();
			entities.citizens.resize(citizens_n);

			Random rand(0);
			test_rand = Random(0);

			auto* car_asset = assets.cars[0].get();

			if (entities.buildings.size() > 0) {
				for (int i=0; i<citizens_n; ++i) {
					auto* building = entities.buildings[rand.uniformi(0, (int)entities.buildings.size())].get();
					entities.citizens[i] = std::move( std::make_unique<Citizen>(Citizen{rand, building}) );
					entities.citizens[i]->asset = car_asset;
				}
			}
		}
	}
	
	void update () {
		ZoneScoped;
		
		if (input.buttons[KEY_P].went_down) {
			view_dbg_cam = !view_dbg_cam;
			if (view_dbg_cam) {
				auto tmp_view = cam.clac_view((float2)input.window_size);
				dbg_cam.rot_aer = cam.rot_aer;
				dbg_cam.pos = tmp_view.cam_pos;

				dbg_cam_cursor_was_enabled = input.cursor_enabled;
				input.set_cursor_mode(*this, false);
			}
			else {
				input.set_cursor_mode(*this, dbg_cam_cursor_was_enabled);
			}
		}
		

		if (input.buttons[KEY_SPACE].went_down)
			sim_paused = !sim_paused;

		spawn();

		if (!day_pause) day_t = wrap(day_t + day_speed * input.dt, 1.0f);
		sun_dir = rotate3_Z(sun_azim) * rotate3_X(sun_elev) * rotate3_Y(day_t * deg(360)) * float3(0,0,-1);

		view = view_dbg_cam ?
			dbg_cam.update(input, (float2)input.window_size) :
			cam.update(input, (float2)input.window_size);

		test.update(input, view);

		g_dbgdraw.axis_gizmo(view, input.window_size);

		net.simulate(*this);

		// select after updating positions
		update_selection();
	}

	virtual void frame () {
		ZoneScoped;
		
		g_dbgdraw.clear();
		renderer->begin(*this);

		update();

		renderer->end(*this);


		assets.assets_reloaded = false;
		entities.buildings_changed = false;
	}
};

extern inline Engine* new_app () {
	return new App();
}
