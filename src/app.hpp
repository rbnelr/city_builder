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
struct Entities {

	//std::vector<std::unique_ptr<Building>> buildings;
	std::vector<Building> buildings;
	
	bool changes = true;
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
		SERIALIZE_FROM_JSON_EXPAND(cam, assets);
	}

	virtual void json_load () { load("debug.json", this); }
	virtual void json_save () { save("debug.json", *this); }
	
	Assets assets;
	Entities entities;

	float sun_azim = deg(30); // degrees from east, counter clockwise
	float sun_elev = deg(14);
	float day_t = 0.6f; // [0,1] -> [0,24] hours
	float day_speed = 1.0f / 60.0f;
	bool  day_pause = true;

	GameCamera cam = GameCamera{ float3(8,8,0)*1024 };
	View3D view;

	float3 sun_dir;

	std::unique_ptr<Renderer> renderer = create_ogl_backend();

	void create_nxn_buildings () {
		static int n = 10;

		if (ImGui::SliderInt("n", &n, 1, 1000) || assets.assets_reloaded) {
			entities.buildings.clear();

			auto pos = float3(8,8,0)*1024;
			auto* house0 = assets.buildings[0].get();
			auto* house1 = assets.buildings[1].get();

			for (int y=0; y<n; ++y) {
				Random rand(y); // y as hash to get deterministic results on resize, opt -> this is a lot faster than doing Random rand(hash(int2(x,y))) for each xy
				for (int x=0; x<n; ++x) {
					auto* asset = rand.uniformi(0, 2) ? house0 : house1;
					//entities.buildings.push_back(std::make_unique<Building>(Building{ asset, pos + float3((float)x,(float)y,0) * float3(40,25,0) }));
					entities.buildings.push_back({ asset, pos + float3((float)x,(float)y,0) * float3(40,25,0) }); // a lot faster
				}
			}

			entities.changes = true;
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

		for (auto& building : entities.buildings) {
			//renderer->dbgdraw.quad(float3(building->pos), (float2)building->asset->size, lrgba(1,1,1,1));
			//renderer->dbgdraw.wire_quad(float3(building->pos), (float2)building->asset->size, lrgba(1,1,1,1));
		}
	}

	virtual void frame () {
		ZoneScoped;
		
		renderer->begin(*this);

		update();

		renderer->end(*this);


		assets.assets_reloaded = false;
		entities.changes = false;
	}
};
