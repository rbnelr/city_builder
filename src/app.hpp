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

	virtual void to_json (nlohmann::ordered_json& j) = 0;
	virtual void from_json (nlohmann::ordered_json const& j) = 0;
};

std::unique_ptr<Renderer> create_ogl_backend ();


inline int length2int (float len) {
	return ceili(len * 100.0f);
}
inline constexpr float LANE_COLLISION_R = 1.3f;
inline constexpr float CAR_SIZE = 3.5f;

inline constexpr float SAFETY_DIST = 1.0f;


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
	std::unique_ptr<network::Agent> agent = nullptr;

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
	
	friend SERIALIZE_TO_JSON(App) {
		SERIALIZE_TO_JSON_EXPAND(cam, assets, net, time_of_day, sim_paused, sim_speed,
			_grid_n, _citizens_n);
		t.renderer->to_json(j);
	}
	friend SERIALIZE_FROM_JSON(App) {
		SERIALIZE_FROM_JSON_EXPAND(cam, assets, net, time_of_day, sim_paused, sim_speed,
			_grid_n, _citizens_n);
		t.renderer->from_json(j);
	}

	virtual void json_load () { deserialize("debug.json", this); }
	virtual void json_save () { serialize("debug.json", *this); }

	virtual void imgui () {
		ZoneScoped;

		renderer->imgui(*this);

		ImGui::Separator();

		settings.imgui();

		cam.imgui("cam");
		dbg_cam.imgui("dbg_cam");
		ImGui::SameLine();
		ImGui::Checkbox("View", &view_dbg_cam);

		time_of_day.imgui();

		ImGui::Checkbox("sim_paused", &sim_paused);
		ImGui::SliderFloat("sim_speed", &sim_speed, 0, 10);

		assets.imgui(settings);

		net.imgui();
	}

	Settings settings;
	
	Assets assets;
	Entities entities;
	network::Network net;

	GameCamera cam = GameCamera{ float3(8,8,0) * 1024 };
	View3D view;

	Flycam dbg_cam = Flycam(0, 0, 100);
	bool view_dbg_cam = false;
	bool dbg_cam_cursor_was_enabled;

	struct TimeOfDay {
		SERIALIZE(TimeOfDay, sun_azim, sun_elev, day_t, day_speed, day_pause)

		float sun_azim = deg(50); // degrees from east, counter clockwise
		float sun_elev = deg(14);
		float day_t = 0.6f; // [0,1] -> [0,24] hours
		float day_speed = 1.0f / 60.0f;
		bool  day_pause = true;

		float3 sun_dir;

		float3x3 sun2world;
		float3x3 world2sun;

		void imgui () {
			if (ImGui::TreeNode("Time of Day")) {

				ImGui::SliderAngle("sun_azim", &sun_azim, 0, 360);
				ImGui::SliderAngle("sun_elev", &sun_elev, -90, 90);
				ImGui::SliderFloat("day_t", &day_t, 0,1);

				ImGui::SliderFloat("day_speed", &day_speed, 0, 0.25f, "%.3f", ImGuiSliderFlags_Logarithmic);
				ImGui::Checkbox("day_pause", &day_pause);
			
				ImGui::TreePop();
			}
		}

		void update (App& app) {
			if (!day_pause) day_t = wrap(day_t + day_speed * app.input.dt, 1.0f);

			// move ang into [-0.5, +0.5] range to make default sun be from top
			// (can use sun2world matrix with -Z facing camera to render shadow map, instead of having wierd camera from below)
			float ang = wrap(day_t - 0.5f, 0.0f, 1.0f) * deg(360);

			// sun rotates from east (+X) to west (-X) -> CW around Y with day_t=0 => midnight, ie sun at -Z
			
			sun2world = rotate3_Z(sun_azim) * rotate3_X(sun_elev) * rotate3_Y(-ang);
			world2sun = rotate3_Y(ang) * rotate3_X(-sun_elev) * rotate3_Z(-sun_azim);

			sun_dir = sun2world * float3(0,0,-1);
		}
	};
	TimeOfDay time_of_day;

	std::unique_ptr<Renderer> renderer = create_ogl_backend();

	bool sim_paused = false;
	float sim_speed = 1;

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

	
	int _grid_n = 10;
	int _citizens_n = 600;

	float _intersection_radius = 0.0f;

	float _connection_chance = 0.7f;

	void spawn () {
		using namespace network;

		bool buildings = ImGui::SliderInt("grid_n", &_grid_n, 1, 1000)
			|| assets.assets_reloaded;
		buildings = ImGui::Button("Respawn buildings") || buildings;

		bool citizens  = ImGui::SliderInt("citizens_n",  &_citizens_n,  0, 1000)
			|| assets.assets_reloaded || buildings;
		citizens = ImGui::Button("Respawn Citizens") || citizens;

		ImGui::SliderFloat("intersection_radius", &_intersection_radius, 0, 30);

		ImGui::SliderFloat("connection_chance", &_connection_chance, 0, 1);

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

			net.nodes.resize((_grid_n+1)*(_grid_n+1));
			
			auto get_node = [&] (int x, int y) -> Node* {
				return net.nodes[y * (_grid_n+1) + x].get();
			};
			
			auto* small_road  = assets.networks[0].get();
			auto* medium_road = assets.networks[1].get();
			
			float2 spacing = float2(60, 60);

			auto road_type = [&] (int x_or_y) {
				return (x_or_y-5) % 10 == 0 ? medium_road : small_road;
			};

			// create path nodes grid
			for (int y=0; y<_grid_n+1; ++y)
			for (int x=0; x<_grid_n+1; ++x) {
				float3 pos = base_pos + float3((float)x,(float)y,0) * float3(spacing, 0);
				net.nodes[y * (_grid_n+1) + x] = std::make_unique<Node>(Node{pos});
			}
			
			auto create_segment = [&] (NetworkAsset* layout, Node* node_a, Node* node_b, int pos) {
				assert(node_a && node_b && node_a != node_b);

				float3 dir = normalizesafe(node_b->pos - node_a->pos);

				auto* seg = net.segments.emplace_back(std::make_unique<Segment>(Segment{
					layout, node_a, node_b
				})).get();

				node_a->segments.push_back(seg);
				node_b->segments.push_back(seg);

				seg->agents.lanes.resize(layout->lanes.size());

				seg->pos_a = node_a->pos + dir * (road_type(pos  )->width * 0.5f + _intersection_radius);
				seg->pos_b = node_b->pos - dir * (road_type(pos+1)->width * 0.5f + _intersection_radius);

				seg->update_cached();
			};

			// create x paths
			for (int y=0; y<_grid_n+1; ++y)
			for (int x=0; x<_grid_n; ++x) {
				auto layout = road_type(y);

				auto* a = get_node(x, y);
				auto* b = get_node(x+1, y);
				create_segment(layout, a, b, x);
			}
			// create y paths
			for (int y=0; y<_grid_n; ++y)
			for (int x=0; x<_grid_n+1; ++x) {
				auto layout = road_type(x);

				if (rand.chance(_connection_chance) || layout == medium_road) {
					auto* a = get_node(x, y);
					auto* b = get_node(x, y+1);
					create_segment(layout, a, b, y);
				}
			}

			for (auto& node : net.nodes) {
				node->update_cached(); // update seg connections
			}

			for (int y=0; y<_grid_n+1; ++y)
			for (int x=0; x<_grid_n; ++x) {
				Random rand(hash(int2(x,y))); // position-based rand
				auto* asset = rand.uniformi(0, 2) ? house0 : house1;

				float3 road_center = (float3((float)x,(float)y,0) + float3(0.5f,0,0)) * float3(spacing,0);

				float3 pos1 = base_pos + road_center + float3(0, asset->size.y, 0);
				float rot1 = deg(90);
				auto build1 = std::make_unique<Building>(Building{ asset, pos1, rot1 });
				
				float3 pos2 = base_pos + road_center - float3(0, asset->size.y, 0);
				float rot2 = deg(-90);
				auto build2 = std::make_unique<Building>(Building{ asset, pos2, rot2 });

				//
				auto* a = get_node(x, y);
				auto* b = get_node(x+1, y);
				for (auto& seg : a->segments) {
					auto* other_node = seg->node_a != a ? seg->node_a : seg->node_b;
					if (other_node == b) {
						// found path in front of building
						build1->connected_segment = seg;
						build2->connected_segment = seg;
						break;
					}
				}

				assert(build1->connected_segment);
				assert(build2->connected_segment);
				entities.buildings.emplace_back(std::move(build1));
				entities.buildings.emplace_back(std::move(build2));
			}

			entities.buildings_changed = true;
		}
		
		if (citizens) {
			ZoneScopedN("spawn citizens");

			clear_sel<Citizen*>();

			// remove references
			for (auto& node : net.nodes) {
				node->agents.free.list.clear();
				node->agents.test.list.clear();
			}
			for (auto& seg : net.segments) {
				for (auto& lane : seg->agents.lanes) {
					lane.list.list.clear();
				}
			}

			entities.citizens.clear();
			entities.citizens.resize(_citizens_n);

			Random rand(0);
			test_rand = Random(0);

			auto* car_asset = assets.cars[0].get();

			if (entities.buildings.size() > 0) {
				for (int i=0; i<_citizens_n; ++i) {
					auto* building = entities.buildings[rand.uniformi(0, (int)entities.buildings.size())].get();
					entities.citizens[i] = std::make_unique<Citizen>(Citizen{rand, building});
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

		time_of_day.update(*this);
		
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
