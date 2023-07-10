#pragma once
#include "common.hpp"
#include "engine/camera.hpp"

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

	struct Binds {
		Button rotate      = MOUSE_BUTTON_RIGHT;
		Button drag        = MOUSE_BUTTON_MIDDLE;
		
		Button move_left   = KEY_A;
		Button move_right  = KEY_D;
		Button move_back   = KEY_S;
		Button move_forw   = KEY_W;
		Button move_up     = KEY_HOME;
		Button move_down   = KEY_END;

		//Button rot_left    = KEY_Q;
		//Button rot_right   = KEY_E;
		Button rot_left    = KEY_NULL;
		Button rot_right   = KEY_NULL;

		Button zoom_in     = KEY_KP_ADD;
		Button zoom_out    = KEY_KP_SUBTRACT;

		float sensitivity = deg(100) / 1000;
	};
	
	Binds binds;

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

	View3D update (Input& I, float2 const& viewport_size) {

		bool scroll_fps = I.buttons[KEY_F].is_down;

		//// look
		{
			float& azimuth   = rot_aer.x;
			float& elevation = rot_aer.y;
			float& roll      = rot_aer.z;

			// Mouselook
			auto raw_mouselook = I.mouse_delta;

			float2 delta = 0;
			if (I.buttons[binds.rotate].is_down)
				delta = raw_mouselook * binds.sensitivity;

			azimuth   -= delta.x;
			elevation += delta.y;

			azimuth = wrap(azimuth, deg(-180), deg(180));
			elevation = clamp(elevation, deg(-90), deg(85));

			// roll with keys
			//float roll_dir = 0;
			//if (I.buttons[KEY_Q].is_down) roll_dir += 1;
			//if (I.buttons[KEY_E].is_down) roll_dir -= 1;
			//
			//float roll_speed = deg(90);
			//
			//roll += roll_dir * roll_speed * I.real_dt;
			//roll = wrap(roll, deg(-180), deg(180));
		}

		float3x3 cam2world_rot, world2cam_rot;
		azimuthal_mount(rot_aer, &world2cam_rot, &cam2world_rot);
		
		float distance;
		{ //// zoom
			// key zoom
			float zoom_dir = 0;
			if (binds.zoom_in  >= 0 && I.buttons[binds.zoom_in ].is_down) zoom_dir += 1;
			if (binds.zoom_out >= 0 && I.buttons[binds.zoom_out].is_down) zoom_dir -= 1;

			float zoom_delta = zoom_dir * zoom_speed*8.0f * I.real_dt;

			// mousewheel zoom
			zoom_delta += (float)I.mouse_wheel_delta * zoom_speed;
			if (!scroll_fps)
				zoom_target += zoom_delta;

			zoom = smooth_var(I.real_dt, zoom,    zoom_target,    zoom_smooth_fac, 1);

			distance = powf(2.0f, -zoom);
		}

		{ //// movement
			float3 move_dir = 0;
			if (I.buttons[binds.move_left ].is_down) move_dir.x -= 1;
			if (I.buttons[binds.move_right].is_down) move_dir.x += 1;
			if (I.buttons[binds.move_forw ].is_down) move_dir.z -= 1;
			if (I.buttons[binds.move_back ].is_down) move_dir.z += 1;
			if (I.buttons[binds.move_down ].is_down) move_dir.y -= 1;
			if (I.buttons[binds.move_up   ].is_down) move_dir.y += 1;

			move_dir = normalizesafe(move_dir);
			float move_speed = length(move_dir); // could be analog with gamepad

			if (move_speed == 0.0f)
				cur_speed = base_speed; // no movement resets speed

			if (I.buttons[KEY_LEFT_SHIFT].is_down) {
				move_speed *= fast_multiplier;

				cur_speed += base_speed * speedup_factor * I.unscaled_dt;
			}

			cur_speed = clamp(cur_speed, base_speed, max_speed);

			float3 delta_cam = cur_speed * distance * move_dir * I.unscaled_dt;

			float2 delta2 = rotate2(rot_aer.x) * float2(delta_cam.x, -delta_cam.z);
			orbit_pos.x += delta2.x;
			orbit_pos.y += delta2.y;
			orbit_pos.z += delta_cam.y;
		}

		{ //// speed or fov change with mousewheel
			if (scroll_fps) {
				float delta_log = -0.1f * I.mouse_wheel_delta;
				vfov = clamp(powf(2.0f, log2f(vfov) + delta_log), deg(1.0f/10), deg(170));
			}
		}

		float aspect = viewport_size.x / viewport_size.y;
		float2 frust_size;

		View3D view;
		// P matrices
		persp_cam2clip(vfov, aspect, clip_near, clip_far, &view.cam2clip, &view.clip2cam, &frust_size);
		// V matrices
		float3 _pos;
		{
			float3 _dir = cam2world_rot * float3(0,0,1);
			_pos = orbit_pos + _dir * distance;
			_pos.z = max(_pos.z, 0.01f);
		}
		view.world2cam = float4x4(world2cam_rot) * float4x4(translate(-_pos));
		view.cam2world = float4x4(translate(_pos)) * float4x4(cam2world_rot);
		// VP inverses
		view.world2clip = view.cam2clip * view.world2cam;
		view.clip2world = view.cam2world * view.clip2cam;
		// misc
		view.frust_near_size    = frust_size * 2.0f;
		view.clip_near          = clip_near;
		view.clip_far           = clip_far;
		view.cam_pos            = _pos;
		view.aspect_ratio       = aspect;
		view.viewport_size      = viewport_size;
		view.inv_viewport_size  = 1.0f / viewport_size;
		return view;
	}
};
