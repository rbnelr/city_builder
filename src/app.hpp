#pragma once
#include "common.hpp"
#include "engine/engine.hpp"
#include "game_camera.hpp"
#include "assets.hpp"

struct App;

struct Renderer {

	virtual ~Renderer () {}
	
	virtual void imgui (App& app) = 0;

	virtual void begin (App& app) = 0;
	virtual void end (App& app) = 0;

	render::DebugDraw dbgdraw;
};

std::unique_ptr<Renderer> create_ogl_backend ();

struct Building {
	BuildingAsset* asset;

	float3 pos = 0;
	float  rot = 0;
};
struct Citizen {
	// TODO: needs to be some kind of state like in car, or in building
	// OR car/building etc needs to track citizen and we dont know where the citizen is
	// probably best to first use double pointers everywhere, likely that this is not a problem in terms of memory
	float3 pos = 0;
	Building* current_target = nullptr;


	Building* home = nullptr;
	Building* work = nullptr;

	lrgb col;

	Citizen (Random& r) {
		col = hsv2rgb(r.uniformf(), 1.0f, 0.8f);
	}
};

struct Network {

	struct Path;
	struct Node {
		float3 pos;
		std::vector<Path*> paths;
	};

	struct Path {
		Node* a;
		Node* b;
	};

	std::vector<std::unique_ptr<Node>> nodes;
	std::vector<std::unique_ptr<Path>> paths;
};

struct Entities {

	std::vector<std::unique_ptr<Building>> buildings;
	std::vector<std::unique_ptr<Citizen>> citizens;
};

struct App : public Engine {

	App (): Engine{"Kiss-Framework Project"} {
		//auto pos = float3(8,8,0)*1024;
		//auto* house = assets.buildings[0].get();
		//entities.buildings.push_back(std::make_unique<Building>(Building{ house, pos + float3(0,0,0) }));
		//entities.buildings.push_back(std::make_unique<Building>(Building{ house, pos + float3(32,0,0) }));
		//entities.buildings.push_back(std::make_unique<Building>(Building{ house, pos + float3(32,32,0) }));
	}
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

	void create_nxn_buildings () {
		static int n = 10;

		if (ImGui::SliderInt("n", &n, 1, 1000) || assets.assets_reloaded) {
			entities = {};
			net = {};

			auto base_pos = float3(8,8,0)*1024;
			auto* house0 = assets.buildings[0].get();
			auto* house1 = assets.buildings[1].get();

			net.nodes.resize((n+1)*(n+1));
			
			auto get_node = [&] (int x, int y) -> Network::Node* {
				return net.nodes[y * (n+1) + x].get();
			};
			
			// create path nodes grid
			for (int y=0; y<n+1; ++y)
			for (int x=0; x<n+1; ++x) {
				float3 pos = base_pos + float3((float)x,(float)y,0) * (float3(40,25,0) + street_size);
				net.nodes[y * (n+1) + x] = std::make_unique<Network::Node>(Network::Node{pos});
			}

			// create x paths
			for (int y=0; y<n+1; ++y)
			for (int x=0; x<n; ++x) {
				auto& path = net.paths.emplace_back(std::make_unique<Network::Path>(Network::Path{}));
				path->a = get_node(x, y);
				path->b = get_node(x+1, y);
			}
			// create y paths
			for (int y=0; y<n; ++y)
			for (int x=0; x<n+1; ++x) {
				auto& path = net.paths.emplace_back(std::make_unique<Network::Path>(Network::Path{}));
				path->a = get_node(x, y);
				path->b = get_node(x, y+1);
			}
			
			for (int y=0; y<n; ++y)
			for (int x=0; x<n; ++x) {
				Random rand(hash(int2(x,y))); // position-based rand
				auto* asset = rand.uniformi(0, 2) ? house0 : house1;

				float3 pos = base_pos + (float3((float)x,(float)y,0) + float3(0.5f)) * (float3(40,25,0) + street_size);
				entities.buildings.push_back(std::make_unique<Building>(Building{ asset, pos }));
			}

			Random rand(0);
			for (int i=0; i<20; ++i) {
				auto& cit = entities.citizens.emplace_back(std::make_unique<Citizen>(Citizen{rand}));

				if (entities.buildings.size() > 0) {
					cit->current_target = entities.buildings[0].get();
					cit->pos = cit->current_target->pos;
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

	void update () {
		ZoneScoped;

		create_nxn_buildings();

		if (!day_pause) day_t = wrap(day_t + day_speed * input.dt, 1.0f);
		sun_dir = rotate3_Z(sun_azim) * rotate3_X(sun_elev) * rotate3_Y(day_t * deg(360)) * float3(0,0,-1);

		renderer->dbgdraw.clear();

		view = cam.update(input, (float2)input.window_size);

		renderer->dbgdraw.axis_gizmo(view, input.window_size);

		static float speed = 50;
		ImGui::SliderFloat("speed", &speed, 0, 100, "%.3f", ImGuiSliderFlags_Logarithmic);

		for (auto& building : entities.buildings) {
			//renderer->dbgdraw.quad(float3(building->pos), (float2)building->asset->size, lrgba(1,1,1,1));
			//renderer->dbgdraw.wire_quad(float3(building->pos), (float2)building->asset->size, lrgba(1,1,1,1));
		}
		for (auto& cit : entities.citizens) {
			if (!cit->current_target) {
				cit->current_target = entities.buildings[ random.uniformi(0, (int)entities.buildings.size()) ].get();
			}

			float3 offset = cit->current_target->pos - cit->pos;
			float dist = length(offset);
			if (dist <= 0.01f) {
				cit->pos = cit->current_target->pos;
				cit->current_target = nullptr;
			}
			else {
				float3 vel = offset / dist * min(dist, speed * input.dt);
				cit->pos += vel;
			}

			float rad = cit == entities.citizens[0] ? 5 : 0.3f * 5;
			renderer->dbgdraw.cylinder(cit->pos, rad, 1.7f, lrgba(cit->col, 1));
		}

		if (entities.citizens.size() > 0) {
			auto* targ = entities.citizens[0]->current_target;
			if (targ) {
				float2 size = (float2)targ->asset->size;
				renderer->dbgdraw.wire_quad(float3(targ->pos - float3(size*0.5f,0)), size, lrgba(0,1,0,1));

				float3 a = entities.citizens[0]->pos;
				renderer->dbgdraw.vector(a, targ->pos - a, lrgba(0,1,0,1));
			}
		}
	}

	virtual void frame () {
		ZoneScoped;
		
		renderer->begin(*this);

		update();

		renderer->end(*this);


		assets.assets_reloaded = false;
	}
};
