#pragma once
#include "common.hpp"
#include "engine/engine.hpp"
#include "game_camera.hpp"
#include "assets.hpp"
#include "util.hpp"
#include "network.hpp"
#include "heightmap.hpp"

struct App;

struct Renderer {

	virtual ~Renderer () {}
	
	virtual void imgui (App& app) = 0;

	virtual void reload_textures () = 0;

	virtual void begin (App& app) = 0;
	virtual void end (App& app, View3D& view) = 0;

	virtual void to_json (nlohmann::ordered_json& j) = 0;
	virtual void from_json (nlohmann::ordered_json const& j) = 0;
};

std::unique_ptr<Renderer> create_ogl_backend ();

inline int length2int (float len) {
	return ceili(len * 100.0f);
}
inline constexpr float LANE_COLLISION_R = 1.3f;

inline constexpr float SAFETY_DIST = 1.0f;


struct Person;
struct Building;

typedef NullableVariant<Person*, network::Node*> sel_ptr;

struct Building {
	BuildingAsset* asset;

	float3 pos = 0;
	float  rot = 0;

	network::Segment* connected_segment = nullptr;
};

struct Person {
	// TODO: needs to be some kind of state like in car, or in building
	// OR car/building etc needs to track citizen and we dont know where the citizen is
	// probably best to first use double pointers everywhere, likely that this is not a problem in terms of memory

	//Building* home = nullptr;
	//Building* work = nullptr;
	
	Building* cur_building = nullptr;
	float stay_timer = 0;

	// while driving this is non null and represents the currently driving car
	// (while inside building car does not exist)
	std::unique_ptr<network::ActiveVehicle> vehicle = nullptr;

	VehicleAsset* owned_vehicle;

	lrgb col;
	float agressiveness;

	float topspeed_accel_mul () {
		return clamp(1.1f + agressiveness, 0.7f, 1.5f);
	}

	Person (Random& r, Building* initial_building, VehicleAsset* owned_vehicle) {
		this->cur_building = initial_building;
		this->owned_vehicle = owned_vehicle;

		bool van = owned_vehicle->mesh_filename == "vehicles/van.fbx";
		bool bus = owned_vehicle->mesh_filename == "vehicles/bus.fbx";

		if (r.chance(0.5f) && !bus) {
			lrgb std_colors[] = {
				lrgb(0,0,0),
				lrgb(0,0,0),
				lrgb(0.1f,0.1f,0.1f),
				lrgb(0.5f,0.5f,0.55f),
				lrgb(1,1,1),
				lrgb(0.95f,0.1f,0.1f),
			};
			col = std_colors[r.uniformi(0, ARRLEN(std_colors))];
		}
		else {
			col = hsv2rgb(r.uniformf(), 1.0f, 0.8f); // debug-like colorful colors
		}

		if (van && r.chance(0.8f)) {
			col = lrgb(1,1,1);
		}

		float agressiveness_deviation = 0.15f;
		agressiveness = r.normalf(agressiveness_deviation, 0.0f);

		stay_timer = r.uniformf(0,1);
	}

	bool selectable () {
		// can only select while driving currently
		return vehicle != nullptr;
	}
	SelCircle get_sel_shape () {
		auto c = lrgb(0.04f, 1, 0.04f);
		if (vehicle)
			return { vehicle->center(), vehicle->car_len()*0.5f, c };
		return { cur_building->pos, 1, c };
	}
};

struct Entities {

	std::vector<std::unique_ptr<Building>> buildings;
	std::vector<std::unique_ptr<Person>> persons;

	// building and streets
	bool buildings_changed = true;
};

struct Test {
	SERIALIZE(Test, bez, speed)

	struct DraggableBezier3 : public Bezier3 {

		void update (Input& I, View3D& view, float sel_r) {
			ImGui::DragFloat2("a", &a.x, 1);
			ImGui::DragFloat2("b", &b.x, 1);
			ImGui::DragFloat2("c", &c.x, 1);
			//ImGui::DragFloat2("d", &d.x, 1);

			draggable(I, view, &a, sel_r);
			draggable(I, view, &b, sel_r);
			draggable(I, view, &c, sel_r);
			//draggable(I, view, &d, sel_r);
		
			//g_dbgdraw.point(float3(a,0), sel_r, lrgba(1,0,0,1));
			//g_dbgdraw.point(float3(b,0), sel_r, lrgba(1,1,0,1));
			//g_dbgdraw.point(float3(c,0), sel_r, lrgba(0,1,0,1));
			//g_dbgdraw.point(float3(d,0), sel_r, lrgba(0,0,1,1));
		}

		void draw_lane (View3D& view, int n, float w, float z, lrgba col) {
			auto prev = eval(0);

			float2 left = rotate90(normalize(prev.vel)) * w*0.5f;
			float2 prevL = prev.pos + left;
			float2 prevR = prev.pos - left;

			for (int i=0; i<n; ++i) {
				float t = (float)(i+1) / (float)n;
				
				auto val = eval(t);

				float2 left = rotate90(normalize(val.vel)) * w*0.5f;
				float2 L = val.pos + left;
				float2 R = val.pos - left;
			
				
				if (i < n-1) {
					g_dbgdraw.line(float3(prev.pos, z), float3(val.pos, z), col);
				}
				else {
					g_dbgdraw.arrow(view, float3(prev.pos, z), float3(val.pos - prev.pos, z), 1, col);
				}

				g_dbgdraw.line(float3(prevL, z), float3(L, z), col);
				g_dbgdraw.line(float3(prevR, z), float3(R, z), col);

				prev = val;
				prevL = L;
				prevR = R;
			}
		}
	};

	DraggableBezier3 bez = {
		float2(0, 0),
		float2(20, 20),
		float2(0, 20),
		//float2(20, 0),
	};

	float sel_r = 2;

	float lane_width = 3.2f;
	float2 car_size = float2(1.4f, 3.5f);

	float t = 0;
	float speed = 2;

	float car_steer = 0; // -deg(30)
	float2 car_center = 0;
	float  car_rot = 0;

	void draw_car (View3D& view, float2 pos, float ang, float steer_curv) {

		float steer_radius = 1.0f / steer_curv;
		
		float2 forw  = rotate2(ang) * float2(0,1);
		float2 right = rotate90(-forw);

		float2 rear_pos   = pos - forw * car_size.y * 0.5f;
		float2 front_posL = rear_pos + forw * car_size.y - right * car_size.x * 0.5f;
		float2 front_posR = rear_pos + forw * car_size.y + right * car_size.x * 0.5f;

		float2 turn_circ_center = rear_pos + right * steer_radius;
		g_dbgdraw.wire_circle(float3(turn_circ_center, 0), steer_radius, lrgba(0,1,1,1));

		{ // draw body
			float3 a = float3(pos - right*car_size.x*0.6f - forw*car_size.y*0.6f, 0);
			float3 b = float3(pos + right*car_size.x*0.6f - forw*car_size.y*0.6f, 0);
			float3 c = float3(pos + right*car_size.x*0.6f + forw*car_size.y*0.6f, 0);
			float3 d = float3(pos - right*car_size.x*0.6f + forw*car_size.y*0.6f, 0);

			g_dbgdraw.line(a, b, lrgba(1,1,0,1));
			g_dbgdraw.line(b, c, lrgba(1,1,0,1));
			g_dbgdraw.line(c, d, lrgba(1,1,0,1));
			g_dbgdraw.line(d, a, lrgba(1,1,0,1));
		}
		
		g_dbgdraw.line(float3(rear_pos, 0),   float3(turn_circ_center, 0), lrgba(0,1,1,1));
		g_dbgdraw.line(float3(front_posL, 0), float3(turn_circ_center, 0), lrgba(0,1,1,1));
		g_dbgdraw.line(float3(front_posR, 0), float3(turn_circ_center, 0), lrgba(0,1,1,1));

		//float2 steer_forwL = rotate90(normalize(front_posL - turn_circ_center));
		//float2 steer_forwR = rotate90(normalize(front_posR - turn_circ_center));
		//if (steer_radius > 0) {
		//	steer_forwL = -steer_forwL;
		//	steer_forwR = -steer_forwR;
		//}
		
		float steer_angL = atanf(car_size.y / (car_size.x * -0.5f - steer_radius));
		float steer_angR = atanf(car_size.y / (car_size.x * +0.5f - steer_radius));
		float2 steer_forwL = rotate2(steer_angL) * forw;
		float2 steer_forwR = rotate2(steer_angR) * forw;

		{ // draw wheels
			float3 bl = float3(pos - right*car_size.x*0.5f - forw*car_size.y*0.5f, 0);
			float3 br = float3(pos + right*car_size.x*0.5f - forw*car_size.y*0.5f, 0);
			float3 fl = float3(pos - right*car_size.x*0.5f + forw*car_size.y*0.5f, 0);
			float3 fr = float3(pos + right*car_size.x*0.5f + forw*car_size.y*0.5f, 0);

			g_dbgdraw.arrow(view, bl, 2 * float3(forw, 0),        0.3f, lrgba(0,1,0,1));
			g_dbgdraw.arrow(view, br, 2 * float3(forw, 0),        0.3f, lrgba(0,1,0,1));
			g_dbgdraw.arrow(view, fl, 2 * float3(steer_forwL, 0), 0.3f, lrgba(0,1,0,1));
			g_dbgdraw.arrow(view, fr, 2 * float3(steer_forwR, 0), 0.3f, lrgba(0,1,0,1));
		}
	}

	void update (Input& I, View3D& view) {
		{
			bez.update(I, view, sel_r);
		
			bez.draw_lane(view, 64, lane_width, 0, lrgba(1,0,0,1));
			
			//dbg_draw_bez(bez, view, 0, 64, lrgba(1,0,0,1));
		}
		
		ImGui::DragFloat("lane_width", &lane_width, 0.1f);
		ImGui::DragFloat2("car_size", &car_size.x, 0.1f);

		ImGui::SliderFloat("t", &t, 0, 1);
		ImGui::DragFloat("speed", &speed, 0.1f);

		ImGui::SliderFloat("car_steer", &car_steer, -1, 1);
		ImGui::DragFloat2("car_center", &car_center.x, 0.1f);
		ImGui::SliderFloat("car_rot", &car_rot, 0, 2*PI);

		if (I.buttons[KEY_R].is_down) {
			car_center = 0;
			car_rot = 0;
		}

		float2 forw  = rotate2(car_rot) * float2(0,1);
		float2 right = rotate90(-forw);
		
		draw_car(view, car_center, car_rot, car_steer);

		float c = car_steer;
		float d = speed * I.dt; // distance step (distance driven along arc)

		float ang = c * d; // angle step (how much car turns)

		float dx, dy;
		{
			// how much car rear axle moves (in local car space)
			if (abs(c) >= 0.05f) {
				// parametric circle path formula
				dx = (1.0f / c) * (1.0f - cos(ang));
				dy = (1.0f / c) * sin(ang);
			}
			else {
				// use approximation with low (or zero) curvature to avoid div by 0 (and other numerical problems with the trigs?)
				// won't be accurate at all with high d!
				dx = c * d*d / 2.0f;
				dy = d;
			}
		}

		// world space movement
		float2 dp = right * dx + forw * dy;
		
		// move rear axle of car, not center
		float2 rear_axle = car_center - forw * car_size.y * 0.5f;
		rear_axle += dp;
		
		// car direction turns
		car_rot -= ang;

		// new forward vector
		forw  = rotate2(car_rot) * float2(0,1);
		//right = rotate90(-forw);

		// update car center based on new rear axle with new forward vector
		// This is important or otherwise car will turn with center, not rear axle like in real life
		// drive_pos += dp is wrong
		car_center = rear_axle + forw * car_size.y * 0.5f;
	}
};

struct CameraTrack {
	// this implementation is dodgy

	bool track = false;
	bool track_rot = false;

	bool cur_tracking = false;

	float3 offset=0;
	float rot_offset=0;

	void imgui () {
		ImGui::Checkbox("Track Selection", &track);
		ImGui::SameLine();
		ImGui::Checkbox("Track Rotation", &track_rot);
	}

	void update (sel_ptr selection, float3* cam_pos, float* cam_rot) {
		auto* sel = selection.get<Person*>();

		if (track && sel && sel->vehicle) {
			float3 pos;
			float ang;
			sel->vehicle->calc_pos(&pos, &ang);
			
			if (!cur_tracking) {
				//*cam_pos -= pos;
				*cam_pos = 0;
				*cam_rot -= ang;
			}

			offset = pos;
			if (track_rot)
				rot_offset = ang;

			cur_tracking = true;
		}
		else {
			if (cur_tracking) {
				*cam_pos += offset;
				*cam_rot += rot_offset;
				offset = 0;
				rot_offset = 0;
			}

			cur_tracking = false;
		}
	}
};

struct App : public Engine {

	App (): Engine{"City Builder"} {}
	virtual ~App () {}
	
	friend SERIALIZE_TO_JSON(App) {
		SERIALIZE_TO_JSON_EXPAND(cam, assets, net, time_of_day, sim_paused, sim_speed, test,
			_grid_n, _persons_n);
		t.renderer->to_json(j);
	}
	friend SERIALIZE_FROM_JSON(App) {
		SERIALIZE_FROM_JSON_EXPAND(cam, assets, net, time_of_day, sim_paused, sim_speed, test,
			_grid_n, _persons_n);
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

		cam_track.imgui();

		time_of_day.imgui();

		ImGui::Checkbox("sim_paused", &sim_paused);
		ImGui::SliderFloat("sim_speed", &sim_speed, 0, 10);

		assets.imgui(settings);
		
		heightmap.imgui();
		net.imgui();
	}
	
	virtual bool update_files_changed (kiss::ChangedFiles& changed_files) {
		bool success = true;
		
		if (changed_files.any_starts_with("assets")) {
			//assets.reload_all();
			//renderer->reload_textures(changed_files);
		}

		return success;
	}

	float _test_time = 0;

	Settings settings;
	
	Assets assets;

	std::unique_ptr<Renderer> renderer = create_ogl_backend();

	Heightmap heightmap;
	Entities entities;
	network::Network net;

	GameCamera cam = GameCamera{ float3(8,8,0) * 1024 };
	
	CameraTrack cam_track;

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


	bool sim_paused = false;
	float sim_speed = 1;

	float sim_dt () {
		return sim_paused ? 0 : input.dt * sim_speed;
	}

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

	void update_selection (View3D const& view) {
		Ray ray;
		if (!view.cursor_ray(input, &ray.pos, &ray.dir))
			return;

		sel_ptr hover;
		float dist = INF;

		for (auto& person : entities.persons) {
			if (person->selectable()) {
				auto shape = person->get_sel_shape();
				float hit_dist;
				if (shape.test(ray, &hit_dist) && hit_dist < dist) {
					hover = person.get();
					dist = hit_dist;
				}
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
	int _persons_n = 600;

	float _intersection_radius = 0.0f;

	float _connection_chance = 0.7f;

	void spawn () {
		using namespace network;

		bool buildings = ImGui::SliderInt("grid_n", &_grid_n, 1, 1000)
			|| assets.assets_reloaded;
		buildings = ImGui::Button("Respawn buildings") || buildings;

		bool persons  = ImGui::SliderInt("persons_n",  &_persons_n,  0, 1000)
			|| assets.assets_reloaded || buildings;
		persons = ImGui::Button("Respawn Residents") || persons;

		ImGui::SliderFloat("intersection_radius", &_intersection_radius, 0, 30);

		ImGui::SliderFloat("connection_chance", &_connection_chance, 0, 1);
		
		static DistributionPlotter aggress_dist;
		aggress_dist.plot_distribution("vehicle topspeed_accel_mul",
			(int)entities.persons.size(), [&] (int i) { return entities.persons[i]->topspeed_accel_mul(); },
			0.5f, 1.6f, false);

		if (buildings) {
			ZoneScopedN("spawn buildings");

			clear_sel<Node*>();
			//clear_sel<Segment*>();

			entities.buildings.clear();
			net = {};

			Random rand(0);

			auto base_pos = float3(100,100,0);
			auto* house0 = assets.buildings["house"].get();
			auto* house1 = assets.buildings["urban_mixed_use"].get();

			net.nodes.resize((_grid_n+1)*(_grid_n+1));
			
			auto get_node = [&] (int x, int y) -> Node* {
				return net.nodes[y * (_grid_n+1) + x].get();
			};
			
			auto* small_road  = assets.networks["small road"].get();
			auto* medium_road = assets.networks["medium road"].get();
			auto* medium_road_asym = assets.networks["medium road asym"].get();
			
			float2 spacing = float2(60, 60);

			auto road_type = [&] (int2 pos, int axis, bool* flip=nullptr) {
				bool type1 = wrap(pos[axis^1]-5, 0,10) == 0;

				bool type2_0 = wrap(pos[axis]-5, 0,10) <= 1;
				bool type2_1 = wrap(pos[axis]-5, 0,10) >= 8;

				if (type1) {
					if (type2_0 || type2_1) {
						if (flip) *flip = type2_0;
						return medium_road_asym;
					}
					return medium_road;
				}
				return small_road;
			};

			// create path nodes grid
			for (int y=0; y<_grid_n+1; ++y)
			for (int x=0; x<_grid_n+1; ++x) {
				float3 pos = base_pos + float3((float)x,(float)y,0) * float3(spacing, 0);
				auto node = std::make_unique<Node>(Node{pos});

				bool big_intersec = wrap(x-5, 0,10) == 0 && wrap(y-5, 0,10) == 0;
				node->_fully_dedicated_turns = big_intersec;

				net.nodes[y * (_grid_n+1) + x] = std::move(node);
			}
			
			auto create_segment = [&] (NetworkAsset* layout, Node* node_a, Node* node_b, int2 pos, int axis, bool flip) {
				assert(node_a && node_b && node_a != node_b);

				if (flip) std::swap(node_a, node_b);

				float3 dir = normalizesafe(node_b->pos - node_a->pos);

				auto* seg = net.segments.emplace_back(std::make_unique<Segment>(Segment{
					layout, node_a, node_b
				})).get();

				node_a->segments.push_back(seg);
				node_b->segments.push_back(seg);

				int2 pos2 = pos;
				pos2[axis] += 1;

				if (flip) std::swap(pos, pos2);
				
				seg->pos_a = node_a->pos + dir * (road_type(pos,  axis^1)->width * 0.5f + _intersection_radius);
				seg->pos_b = node_b->pos - dir * (road_type(pos2, axis^1)->width * 0.5f + _intersection_radius);

				seg->vehicles.lanes.resize(layout->lanes.size());

				seg->update_cached();
			};

			// create x paths
			for (int y=0; y<_grid_n+1; ++y)
			for (int x=0; x<_grid_n; ++x) {
				bool flip;
				auto layout = road_type(int2(x,y), 0, &flip);

				auto* a = get_node(x, y);
				auto* b = get_node(x+1, y);
				create_segment(layout, a, b, int2(x,y), 0, flip);
			}
			// create y paths
			for (int y=0; y<_grid_n; ++y)
			for (int x=0; x<_grid_n+1; ++x) {
				bool flip;
				auto layout = road_type(int2(x,y), 1, &flip);

				if (layout != small_road || rand.chance(_connection_chance)) {
					auto* a = get_node(x, y);
					auto* b = get_node(x, y+1);
					create_segment(layout, a, b, int2(x,y), 1, flip);
				}
			}

			for (auto& node : net.nodes) {
				node->update_cached();
			}

			for (int y=0; y<_grid_n+1; ++y)
			for (int x=0; x<_grid_n; ++x) {
				Random rand(hash(int2(x,y))); // position-based rand

				Segment* conn_seg = nullptr;
				{
					auto* a = get_node(x, y);
					auto* b = get_node(x+1, y);
					for (auto& seg : a->segments) {
						auto* other_node = seg->node_a != a ? seg->node_a : seg->node_b;
						if (other_node == b) {
							// found path in front of building
							conn_seg = seg;
							break;
						}
					}
				}

				float3 road_center = (float3((float)x,(float)y,0) + float3(0.5f,0,0)) * float3(spacing,0);
				float road_width = conn_seg->asset->width * 0.5f;
				
				{
					auto* asset = rand.uniformi(0, 2) ? house0 : house1;
					float3 pos1 = base_pos + road_center + float3(0, road_width + asset->size.y, 0);
					float rot1 = deg(90);
					auto build1 = std::make_unique<Building>(Building{ asset, pos1, rot1, conn_seg });
					entities.buildings.emplace_back(std::move(build1));
				}
				{
					auto* asset = rand.uniformi(0, 2) ? house0 : house1;
					float3 pos2 = base_pos + road_center - float3(0, road_width + asset->size.y, 0);
					float rot2 = deg(-90);
					auto build2 = std::make_unique<Building>(Building{ asset, pos2, rot2, conn_seg });
					entities.buildings.emplace_back(std::move(build2));
				}
			}

			entities.buildings_changed = true;
		}
		
		if (persons) {
			ZoneScopedN("spawn persons");

			clear_sel<Person*>();

			// remove references
			for (auto& node : net.nodes) {
				node->vehicles.free.list.clear();
				node->vehicles.test.list.clear();
			}
			for (auto& seg : net.segments) {
				for (auto& lane : seg->vehicles.lanes) {
					lane.list.list.clear();
				}
			}

			entities.persons.clear();
			entities.persons.resize(_persons_n);

			Random rand(0);
			auto rand_car = WeightedChoice(assets.vehicles.begin(), assets.vehicles.end(),
				[] (AssetPtr<VehicleAsset> const& car) { return car->spawn_weight; });

			test_rand = Random(0);

			if (entities.buildings.size() > 0) {
				for (int i=0; i<_persons_n; ++i) {
					auto* building = entities.buildings[rand.uniformi(0, (int)entities.buildings.size())].get();
					auto* car_asset = rand_car.get_random(rand)->get();
					
					entities.persons[i] = std::make_unique<Person>(Person{rand, building, car_asset});
				}
			}
		}
	}
	
	View3D update () {
		ZoneScoped;
		
		if (input.buttons[KEY_P].went_down) {
			view_dbg_cam = !view_dbg_cam;
			if (view_dbg_cam) {
				auto tmp_view = cam.clac_view((float2)input.window_size, cam_track.offset, cam_track.rot_offset);
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
		
		cam_track.update(selection, &cam.orbit_pos, &cam.rot_aer.x);

		//view = view_dbg_cam ?
		//	dbg_cam.update(input, (float2)input.window_size) :
		//	cam    .update(input, (float2)input.window_size, cam_track.offset, cam_track.rot_offset);

		net.simulate(*this);
		
		cam_track.update(selection, &cam.orbit_pos, &cam.rot_aer.x);

		View3D view = view_dbg_cam ?
			dbg_cam.update(input, (float2)input.window_size) :
			cam    .update(input, (float2)input.window_size, cam_track.offset, cam_track.rot_offset);
		
		test.update(input, view);

		net.draw_debug(*this, view);

		g_dbgdraw.axis_gizmo(view, input.window_size);

		// select after updating positions
		update_selection(view);

		return view;
	}

	virtual void frame () {
		ZoneScoped;
		
		g_dbgdraw.clear();
		renderer->begin(*this);

		auto view = update();

		renderer->end(*this, view);


		assets.assets_reloaded = false;
		entities.buildings_changed = false;
	}
};

extern inline Engine* new_app () {
	return new App();
}
