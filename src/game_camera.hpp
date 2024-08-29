#pragma once
#include "common.hpp"
#include "engine/camera.hpp"
#include "entities.hpp"

struct GameCamera {
	friend SERIALIZE_TO_JSON(GameCamera) {
		SERIALIZE_TO_JSON_EXPAND(orbit_pos, zoom_target, rot_aer, vfov, clip_near, clip_far)
	}
	friend SERIALIZE_FROM_JSON(GameCamera) {
		SERIALIZE_FROM_JSON_EXPAND(orbit_pos, zoom_target, rot_aer, vfov, clip_near, clip_far)

		// HERE: need this to avoid animated zoom in at load
		t.zoom = t.zoom_target;
	}

	float3 orbit_pos = 0;
	float  zoom = -7; // log2 of orbit distance
	float3 rot_aer = float3(0, deg(-50), 0);

	float  zoom_target = zoom;
	float  zoom_smooth_fac = 16; // 0 to disable
	float  zoom_speed = 0.25f;

	float vfov = deg(50);

	float clip_near = 1.0f/4;
	float clip_far  = 10000;

	float base_speed = 1.0f;
	float max_speed = 10000.0f;
	float speedup_factor = 2;
	float fast_multiplier = 4;

	float cur_speed = 0;
	
	void imgui (const char* label="GameCamera") {
		if (!ImGui::TreeNode(label)) return;

		ImGui::DragFloat3("orbit_pos", &orbit_pos.x, 0.05f);
		
		ImGui::DragFloat("zoom_target", &zoom_target, 0.05f);

		float dist = powf(2.0f, -zoom);
		if (ImGui::DragFloat("distance", &dist, 0.05f, 0.125f, 10*1024, "%.3f", ImGuiSliderFlags_Logarithmic))
			zoom = -log2f(dist);

		float3 rot_deg = to_degrees(rot_aer);
		if (ImGui::DragFloat3("rot_aer", &rot_deg.x, 0.3f))
			rot_aer = to_radians(rot_deg);

		ImGui::SliderAngle("vfov", &vfov, 0,180);

		ImGui::Text("cur_speed: %.3f", cur_speed);

		if (ImGui::TreeNodeEx("details")) {

			ImGui::DragFloat("clip_near", &clip_near, 0.05f);
			ImGui::DragFloat("clip_far" , &clip_far,  0.05f);

			ImGui::DragFloat("base_speed", &base_speed, 0.05f, 0, FLT_MAX, "%.3f", ImGuiSliderFlags_Logarithmic);
			ImGui::DragFloat("max_speed", &max_speed,   0.05f, 0, FLT_MAX, "%.3f", ImGuiSliderFlags_Logarithmic);
			ImGui::DragFloat("speedup_factor", &speedup_factor, 0.001f);
			ImGui::DragFloat("fast_multiplier", &fast_multiplier, 0.05f);

			ImGui::TreePop();
		}

		ImGui::TreePop();
	}

	float calc_orbit_distance () {
		return powf(2.0f, -zoom);
	}
	
	void update (Input& I, float2 const& viewport_size, CameraBinds const& binds) {

		bool scroll_fov = I.buttons[binds.change_fov].is_down;

		rotate_with_mouselook(I, vfov, binds, &rot_aer);
		
		float distance;
		{ //// zoom
			// key zoom
			float zoom_dir = 0;
			if (binds.zoom_in  >= 0 && I.buttons[binds.zoom_in ].is_down) zoom_dir += 1;
			if (binds.zoom_out >= 0 && I.buttons[binds.zoom_out].is_down) zoom_dir -= 1;

			float zoom_delta = zoom_dir * zoom_speed*8.0f * I.real_dt;

			// mousewheel zoom
			zoom_delta += (float)I.mouse_wheel_delta * zoom_speed;
			if (!scroll_fov)
				zoom_target += zoom_delta;

			zoom = smooth_var(I.real_dt, zoom,    zoom_target,    zoom_smooth_fac, 1);

			distance = calc_orbit_distance();
		}

		{ //// movement
			float3 move_dir = binds.get_local_move_dir(I);
			float move_speed = length(move_dir); // could be analog with gamepad

			if (move_speed == 0.0f)
				cur_speed = base_speed; // no movement resets speed

			if (I.buttons[binds.modifier].is_down) {
				move_speed *= fast_multiplier;

				cur_speed += base_speed * speedup_factor * I.real_dt;
			}

			cur_speed = clamp(cur_speed, base_speed, max_speed);

			float3 delta_cam = cur_speed * distance * move_dir * I.real_dt;

			float2 delta2 = rotate2(rot_aer.x) * float2(delta_cam.x, -delta_cam.z);
			orbit_pos.x += delta2.x;
			orbit_pos.y += delta2.y;
			orbit_pos.z += delta_cam.y;
		}

		{ //// speed or fov change with mousewheel
			if (scroll_fov) {
				float delta_log = -0.1f * I.mouse_wheel_delta;
				// TODO: might want to smooth fov change too?
				vfov = clamp(powf(2.0f, log2f(vfov) + delta_log), deg(1.0f/10), deg(170));
			}
		}
	}

	View3D clac_view (float2 const& viewport_size) {

		float3x3 cam2world_rot, world2cam_rot;
		azimuthal_mount(rot_aer, &world2cam_rot, &cam2world_rot);

		float aspect = viewport_size.x / viewport_size.y;
		
		float3 collided_pos;
		{
			float3 _dir = cam2world_rot * float3(0,0,1);
			collided_pos = orbit_pos + _dir * calc_orbit_distance();
			collided_pos.z = max(collided_pos.z, 0.01f);
		}

		float3x4 world2cam = float3x4(world2cam_rot) * translate(-collided_pos);
		float3x4 cam2world = translate(collided_pos) * float3x4(cam2world_rot);

		return persp_view(vfov, aspect, clip_near, clip_far, world2cam, cam2world, viewport_size);
	}
	
	void dbgdraw (View3D& cam_view, lrgba const& col) {
		dbgdraw_frustrum(cam_view.clip2world, col);
		// orbit 
		g_dbgdraw.line(orbit_pos, cam_view.cam_pos, col * 0.5f);
		g_dbgdraw.wire_circle(orbit_pos, 5, col * 0.5f);
	}
};

struct CameraTrack {
	bool track = false;
	bool track_rot = false;

	sel_ptr cur_tracking = nullptr;

	float3 prev_pos;
	float  prev_rot;

	float3 pos_target;

	float smoothing_duration = 1;
	float smoothing_t = 0;

	void imgui () {
		ImGui::Checkbox("Track Selection", &track);
		ImGui::SameLine();
		ImGui::Checkbox("Track Rotation", &track_rot);
	}

	void update (GameCamera& cam, sel_ptr selection, float dt) {
		auto* sel = selection.get<Person*>();

		if (track && sel && sel->vehicle) { // If tracking and object selected
			// get object pos and rotation
			float3 pos;
			float rot;
			sel->vehicle->calc_pos(&pos, &rot);
			
			if (cur_tracking != selection) {
				// re-center camera if new selection or selection changed
				pos_target = pos; // jump to tracking target
				smoothing_t = 0;
			}
			else {
				pos_target += pos - prev_pos;
				if (track_rot)
					cam.rot_aer.x += rot - prev_rot;
			}

			// smoothly lerp from free camera to tracking target for smoothing_duration seconds
			// NOTE: camera movement overridden entirely if CameraTrack::update is called after camera::update
			cam.orbit_pos = lerp(cam.orbit_pos, pos_target, smoothing_t);
			smoothing_t = min(smoothing_t + dt / smoothing_duration, 1.0f);

			prev_pos = pos;
			prev_rot = rot;

			cur_tracking = selection;
		}
		else {
			cur_tracking = nullptr;
		}
	}
};
