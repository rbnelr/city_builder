#pragma once
#include "common.hpp"
#include "engine/engine.hpp"
#include "game_camera.hpp"
#include "assets.hpp"
#include "game_time.hpp"
#include "heightmap.hpp"
#include "network.hpp"
#include "entities.hpp"
#include "interaction.hpp"

class App;

class Renderer {
public:

	virtual ~Renderer () {}
	
	virtual void imgui (App& app) = 0;

	virtual void reload_textures () = 0;

	virtual void begin (App& app) = 0;
	virtual void end (App& app, View3D& view) = 0;

	virtual void to_json (nlohmann::ordered_json& j) = 0;
	virtual void from_json (nlohmann::ordered_json const& j) = 0;
};

std::unique_ptr<Renderer> create_ogl_backend ();

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
		float3(0, 0, 0),
		float3(20, 20, 0),
		float3(0, 20, 0),
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
		float d = speed * I.real_dt; // distance step (distance driven along arc)

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

inline bool ray_cone_intersect (Ray ray, float3 cone_pos, float3 cone_dir, float cone_ang, float2* out_t01) {
	// based on http://lousodrome.net/blog/light/2017/01/03/intersection-of-a-ray-and-a-cone/
	// vector based algebra solution derived from  dot(normalize(pos(t) - cone_pos), cone_dir)^2 == cos(cone_ang)^2

	float cosa = cos(cone_ang);
	
	float3 V = cone_dir;
	float3 D = ray.dir;
	float3 CO = ray.pos - cone_pos;
	
	float D_V = dot(ray.dir, V);
	float CO_V = dot(CO, V);
	float cos2 = cosa * cosa;
	
	// t^2*a + t*b + c = 0
	float a = D_V*D_V - cos2;
	float b = 2.0f * (D_V*CO_V - dot(ray.dir, CO)*cos2);
	float c = CO_V*CO_V - dot(CO, CO)*cos2;
	
	// use pq formula
	if (a == 0.0f) {
		// t*b + c = 0  => t = -c/b
		// what if b == 0?   currently never actually triggers due to a being close to 0 but not equal
		return false; // parallel ray to cone, math doesn't work! TODO: handle this?
	}
	
	float p = b / a * 0.5f;
	float q = c / a;

	float root = p*p - q; // sqrt((p/2)^2 - q)
	if (root < 0.0f)
		return false; // no solutions, ray misses both cones
	root = sqrt(root);
	float t0 = -p - root; // -p/2 +- sqrt((p/2)^2 - q)
	float t1 = -p + root;
	assert(t1 >= t0);
	
	// check which hits hit real or fake (mirror) cone
	bool fake0 = dot(t0 * D + CO, V) < 0.0f;
	bool fake1 = dot(t1 * D + CO, V) < 0.0f;

	if (fake0 && fake1) // both hits on fake cone
		return false;

	if (fake0 || fake1) { // one hit on real cone, one fake
		if (fake0) { // ray towards real cone
			// exited fake cone, entered real cone(at least based on ray dir,
			//  ray might only start inside real and not actually touch fake, but this can later be determined by negative ts)
			t0 = t1; // t1 was actually the entry into real cone
			t1 = INF; // cannot exit out of cone again in this case (or there would be solution for t)
		}
		else { // ray towards fake cone
			// exited real cone, entered fake cone
			t1 = t0; // t0 was actually the exit from real cone
			t0 = -INF; // coming from inside real cone inf far back
		}
	}
	// else: both hits on real cone
	
	assert(t1 >= t0);
	*out_t01 = float2(t0, t1);
	return true;
}
inline void cone_test (View3D& view) {
	struct TestRay {
		float3 pos;
		float3 dir;
		lrgba col;

		TestRay (float3 pos, float elev, float azim, lrgba col) {
			this->pos = pos;
			this->dir = rotate3_Z(azim) * rotate3_X(elev) * float3(0,1,0);
			this->col = col;
		}
	};

	static float3 cone_pos = float3(10, 0, 1);
	static float3 cone_dir = rotate3_X(deg(0)) * float3(0,0,-1);
	static float cone_ang = deg(30);

	static std::vector<TestRay> rays = {
		//{ cone_pos + float3(-.5f,-5,0), deg(-20), deg(1), lrgba(1,0.5f,0.0f,1) },
		//{ cone_pos + float3(+.5f,-5,0), deg(20), deg(1), lrgba(1,0.5f,0.0f,1) },
		//
		//{ cone_pos + float3(-1,-5,1), deg(-30), deg(1), lrgba(1,0.5f,0.5f,1) },
		//{ cone_pos + float3(+1,-5,1), deg(30), deg(1), lrgba(1,0.5f,0.5f,1) },
		//
		//{ cone_pos + float3(-2,-5,-1), deg(-30), deg(1), lrgba(1,0.5f,0.5f,1) },
		//{ cone_pos + float3(+2,-5,-1), deg(30), deg(1), lrgba(1,0.5f,0.5f,1) },
		//
		//{ cone_pos + float3(+2,0,+1), deg(0), deg(90), lrgba(0.5f,1,0.5f,1) },
		//{ cone_pos + float3(+2,0, 0), deg(0), deg(90), lrgba(0.5f,1,0.5f,1) },
		//{ cone_pos + float3(+2,0,-1), deg(0), deg(90), lrgba(0.5f,1,0.5f,1) },
		//
		//{ cone_pos + float3(-2,1,+10), deg( 90), deg(1), lrgba(0.5f,1,0.5f,1) },
		//{ cone_pos + float3(-3,1,+10), deg(-90), deg(1), lrgba(0.5f,1,0.5f,1) },
		//{ cone_pos + float3(-2,0, 0),  deg(-90), deg(1), lrgba(0.5f,1,0.5f,1) },
		//{ cone_pos + float3(-3,0,+1),  deg(-90), deg(1), lrgba(0.5f,1,0.5f,1) },
		//{ cone_pos + float3(-3,3,+1),  deg( 90), deg(1), lrgba(0.5f,1,0.5f,1) },
		//{ cone_pos + float3(-3,2,-10), deg(-90), deg(1), lrgba(0.5f,1,0.5f,1) },
		//{ cone_pos + float3(-2,2,-10), deg( 90), deg(1), lrgba(0.5f,1,0.5f,1) },

		{ cone_pos + float3(0,0, 0), -deg(90)+cone_ang, 0, lrgba(0.5f,0.5f,1,1) },
		{ cone_pos + float3(0,-4,0), -deg(90)+cone_ang, 0, lrgba(0.5f,0.5f,1,1) },
		{ cone_pos + float3(0,+4,0), -deg(90)+cone_ang, 0, lrgba(0.5f,0.5f,1,1) },
		{ cone_pos + float3(-4,0,0), -deg(90)+cone_ang, 0, lrgba(0.5f,0.5f,1,1) },
		{ cone_pos + float3(+4,0,0), -deg(90)+cone_ang, 0, lrgba(0.5f,0.5f,1,1) },

		{ cone_pos + float3(0,0, 0.01f), -deg(90)+cone_ang, 0, lrgba(0.8f,0.5f,0.5f,1) },
		{ cone_pos + float3(0,-4,0.01f), -deg(90)+cone_ang, 0, lrgba(0.8f,0.5f,0.5f,1) },
		{ cone_pos + float3(0,+4,0.01f), -deg(90)+cone_ang, 0, lrgba(0.8f,0.5f,0.5f,1) },
		{ cone_pos + float3(-4,0,0.01f), -deg(90)+cone_ang, 0, lrgba(0.8f,0.5f,0.5f,1) },
		{ cone_pos + float3(+4,0,0.01f), -deg(90)+cone_ang, 0, lrgba(0.8f,0.5f,0.5f,1) },
	};

	g_dbgdraw.wire_cone(cone_pos, cone_ang*2, 20, rotate3_X(deg(180)), lrgba(1,1,1,0.8f), 64, 32);
	g_dbgdraw.wire_cone(cone_pos, cone_ang*2, 20, float3x3::identity(), lrgba(1,1,1,0.3f), 64, 32);

	for (auto& ray : rays) {
		g_dbgdraw.arrow(view, ray.pos, ray.dir * 10, .2f, ray.col);

		float2 t01;
		if (ray_cone_intersect({ ray.pos, ray.dir }, cone_pos, cone_dir, cone_ang, &t01)) {
			float t0 = max(t01.x, 0.0f);
			//float t0 = t01.x;
			float t1 = min(t01.y, 30.0f);

			if (t1 >= t0) {
				float3 a = ray.pos + ray.dir * t0;
				float3 b = ray.pos + ray.dir * t1;
				g_dbgdraw.point(a, 0.1f, lrgba(1,0,0,1));
				g_dbgdraw.point(b, 0.1f, lrgba(0,1,0,1));
			}
		}
	}
};

struct TestMapBuilder {
	SERIALIZE(TestMapBuilder, _grid_n, _persons_n)
	
	int _grid_n = 10;
	int _persons_n = 600;

	float _intersection_radius = 0.0f;

	float _connection_chance = 0.7f;

	void update (Assets& assets, Entities& entities, Network& net, Interaction& interact, Random& sim_rand) {
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

			interact.clear_sel<Node*>();
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
				auto node = std::make_unique<Node>();
				node->pos = base_pos + float3((float)x,(float)y,0) * float3(spacing, 0);

				bool big_intersec = wrap(x-5, 0,10) == 0 && wrap(y-5, 0,10) == 0;
				node->_fully_dedicated_turns = big_intersec;

				net.nodes[y * (_grid_n+1) + x] = std::move(node);
			}
			
			auto create_segment = [&] (NetworkAsset* layout, Node* node_a, Node* node_b, int2 pos, int axis, bool flip) {
				assert(node_a && node_b && node_a != node_b);

				if (flip) std::swap(node_a, node_b);

				float3 dir = normalizesafe(node_b->pos - node_a->pos);

				auto* seg = net.segments.emplace_back(std::make_unique<Segment>()).get();
				seg->asset = layout;
				seg->node_a = node_a;
				seg->node_b = node_b;

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
				node->set_defaults();
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

			interact.clear_sel<Person*>();

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
			
			sim_rand = Random(0);

			auto rand_car = WeightedChoice(assets.vehicles.begin(), assets.vehicles.end(),
				[] (AssetPtr<VehicleAsset> const& car) { return car->spawn_weight; });


			if (entities.buildings.size() > 0) {
				for (int i=0; i<_persons_n; ++i) {
					auto* building = entities.buildings[sim_rand.uniformi(0, (int)entities.buildings.size())].get();
					auto* car_asset = rand_car.get_random(sim_rand)->get();
					
					entities.persons[i] = std::make_unique<Person>(Person{sim_rand, building, car_asset});
				}
			}
		}
	}
};

struct Savefiles {
	static constexpr const char* app_settings_json = "settings.json";
	static constexpr const char* graphics_settings_json = "graphics_settings.json";
	
	template <typename FUNC>
	static inline void load (const char* filepath, FUNC from_json) {
		ZoneScoped;
		try {
			json json;
			if (load_json(filepath, &json)) {
				from_json(json);
			}
		} catch (std::exception& ex) {
			log_error("Error when deserializing something: %s", ex.what());
		}
	}
	template <typename FUNC>
	static inline void save (const char* filepath, FUNC to_json) {
		ZoneScoped;
		try {
			json json;
			to_json(json);
			save_json(filepath, json);
		} catch (std::exception& ex) {
			log_error("Error when serializing something: %s", ex.what());
		}
	}
};

class App : public Engine {
public:

	App (): Engine{"City Builder"} {
		cam_binds = {};
		// RMB is taken for building and terraforming etc.
		cam_binds.rotate = MOUSE_BUTTON_MIDDLE;
		// CTRL/SPACE are nice, but SPACE is standard for pause
		// TODO: maybe actually move up and down with Q/E ?
		cam_binds.move_down = KEY_Q;
		cam_binds.move_up   = KEY_E;
		cam_binds.rot_left  = KEY_NULL;
		cam_binds.rot_right = KEY_NULL;
	}
	virtual ~App () {}
	
	void load_app_settings () {
		Savefiles::load(Savefiles::app_settings_json, [&] (json const& j) { auto& t = *this;
			SERIALIZE_FROM_JSON_EXPAND(assets, options, cam_binds, test, test_map_builder)
		});
	}
	void save_app_settings () {
		Savefiles::save(Savefiles::app_settings_json, [&] (json& j) { auto& t = *this;
			SERIALIZE_TO_JSON_EXPAND(assets, options, cam_binds, test, test_map_builder)
		});
	}
	
	void load_graphics_settings () {
		Savefiles::load(Savefiles::graphics_settings_json, [&] (json const& j) {
			nlohmann::try_get_to(j, "vsync", vsync);
			renderer->from_json(j);

			set_vsync(vsync); // set json-loaded vsync
		});
	}
	void save_graphics_settings () {
		Savefiles::save(Savefiles::graphics_settings_json, [&] (json& j) {
			j["vsync"] = vsync;
			renderer->to_json(j);
		});
	}

	// TODO: refactor this stuff into a map object that you can actually from different files/folders to switch between
	void load_map () {
		Savefiles::load("map.json", [&] (json const& j) { auto& t = *this;
			SERIALIZE_FROM_JSON_EXPAND(time, heightmap, main_cam, network)
		});
	}
	void save_map () {
		Savefiles::save("map.json", [&] (json& j) { auto& t = *this;
			SERIALIZE_TO_JSON_EXPAND(time, heightmap, main_cam, network)
		});
	}

	virtual void json_load () {
		load_app_settings();
		load_graphics_settings();
		load_map();
	}
	virtual void json_save () {
		save_app_settings();
		save_graphics_settings();
		save_map();
	}

	virtual void imgui () {
		ZoneScoped;
		
		main_cam.imgui("main_cam");
		dbg_cam.imgui("dbg_cam");

		ImGui::Checkbox("Debug View [O]", &view_dbg_cam);
		ImGui::Checkbox("Control Main Camera in Debug View [P]", &remote_control_cam);
		ImGui::InputInt("Debug Lod/Cull Mode", &dbg_lodcull);
		
		ImGui::Separator();

		time.imgui();
		interact.imgui(heightmap);

		heightmap.imgui();
		network.imgui();
		
		ImGui::Separator();

		options.imgui();
		assets.imgui(options);

		renderer->imgui(*this);
	}
	
	virtual bool update_files_changed (kiss::ChangedFiles& changed_files) {
		bool success = true;
		
		if (changed_files.any_starts_with("assets")) {
			//assets.reload_all();
			//renderer->reload_textures(changed_files);
		}

		return success;
	}

	Options options;
	
	Assets assets;

	std::unique_ptr<Renderer> renderer = create_ogl_backend();

	GameTime time;

	OverlayDraw overlay;
	
	Interaction interact;
	
	CameraBinds cam_binds;
	GameCamera main_cam = GameCamera{ float3(8,8,0) * 1024 };

	Flycam dbg_cam = Flycam(0, 0, 100);
	bool view_dbg_cam = false;
	bool view_dbg_cam_prev = false;
	bool remote_control_cam = false;
	bool dbg_cam_cursor_was_enabled;
	int dbg_lodcull = 0;

	Heightmap heightmap;
	Entities entities;
	Network network;

	float sim_dt () {
		return time.pause_sim ? 0 : input.real_dt * time.target_gamespeed;
	}

	TestMapBuilder test_map_builder;

	// rand set by TestMapBuilder to get consistent paths for testing
	Random sim_rand;

	Test test;

	View3D update_camera () {
		auto res = (float2)input.window_size;
		
		if (input.buttons[KEY_O].went_down) view_dbg_cam = !view_dbg_cam;
		if (input.buttons[KEY_P].went_down) remote_control_cam = !remote_control_cam;

		if (view_dbg_cam != view_dbg_cam_prev) {
			if (view_dbg_cam) {
				// move dbg cam to position of real camera
				// use previous frame camera (cam not updated yet) to position dbg cam
				auto view = main_cam.clac_view((float2)input.window_size);
				dbg_cam.pos     = view.cam_pos;
				dbg_cam.rot_aer = main_cam.rot_aer;
				dbg_cam.vfov    = main_cam.vfov;

				dbg_cam_cursor_was_enabled = input.cursor_enabled;
				input.set_cursor_mode(*this, false);
			}
			else {
				input.set_cursor_mode(*this, dbg_cam_cursor_was_enabled);
			}
			view_dbg_cam_prev = view_dbg_cam;
		}
		
		//
		interact.cam_track.update(main_cam, interact.selection, input.real_dt);

		View3D view;
		if (!view_dbg_cam) {
			main_cam.update(input, res, cam_binds);
			view = main_cam.clac_view(res);
		}
		else {
			View3D real_view;
			if (remote_control_cam) main_cam.update(input, res, cam_binds);
			else                    dbg_cam .update(input, res, cam_binds);

			real_view = main_cam.clac_view(res);
			view = dbg_cam.clac_view(res);

			//if (view_dbg_cam)
				//main_cam.dbgdraw(real_view, lrgba(1,1,0,1));
		}

		return view;
	}

	View3D dbg_get_main_view () {
		return main_cam.clac_view((float2)input.window_size);
	}

	View3D update () {
		ZoneScoped;
		
		time.update(input);

		test_map_builder.update(assets, entities, network, interact, sim_rand);

		network.simulate(*this);

	////
		View3D view = update_camera();
		
		//test.update(input, view);

		// select after updating positions
		interact.update(heightmap, entities, network, view, input);

		//cone_test(view);
		
	//// Visually relevant
		//time.visualize_planet(view);
		time.calc_sky_config(view);

		network.draw_debug(*this, view);
		
		g_dbgdraw.axis_gizmo(view, input.window_size);
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
