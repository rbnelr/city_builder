#pragma once
#include "common.hpp"
#include "network.hpp"
#include "entities.hpp"

typedef NullableVariant<Person*, network::Node*> sel_ptr;

struct CameraTrack {
	// this implementation is dodgy

	bool track = false;
	bool track_rot = false;

	sel_ptr cur_tracking = nullptr;

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
			
			if (cur_tracking != selection) {
				// re-center camera if new selection or selection changed
				*cam_pos = 0;
				*cam_rot -= ang;
			}

			offset = pos;
			if (track_rot)
				rot_offset = ang;

			cur_tracking = selection;
		}
		else {
			if (cur_tracking) {
				*cam_pos += offset;
				*cam_rot += rot_offset;
				offset = 0;
				rot_offset = 0;
			}

			cur_tracking = nullptr;
		}
	}
};

// TODO: rework this!, probably mirror how heightmap with its tools works
struct Interaction {
	SERIALIZE(Interaction, terraform)

	enum Mode {
		INSPECT=0,
		BUILD,
		BULLDOZE,
		TERRAFORM,
	};
	static inline constexpr const char* Mode_str[] = {
		"Inspect", "Build", "Bulldoze", "Terraform"
	};

	HeightmapTerraform terraform;

	Mode cur_mode = INSPECT;
	
	sel_ptr hover;
	sel_ptr selection;

	CameraTrack cam_track;
	
	bool toggle_traffic_light = false;

	template <typename T>
	void clear_sel () {
		if (hover.get<T>())
			hover = nullptr;
		if (selection.get<T>())
			selection = nullptr;
	}

	void imgui (Heightmap& heightmap) {
		if (!imgui_Header("Interaction", true)) return;

		auto mode_check = [&] (Mode mode) {
			bool active = mode == cur_mode;
			//ImGui::Setstyle
			auto str = prints(active ? "[%s]###%s":"%s###%s", Mode_str[(int)mode], Mode_str[(int)mode]);
			if (ImGui::ButtonEx(str.c_str(), ImVec2(80, 20), ImGuiButtonFlags_PressedOnClick))
				cur_mode = mode;
		};
		mode_check(INSPECT);
		ImGui::SameLine(); // TODO: I would prefer a automatic line-wrap layouting here, can Imgui do that or would I implement that myself?
		mode_check(BUILD);
		ImGui::SameLine();
		mode_check(BULLDOZE);
		//ImGui::SameLine();
		mode_check(TERRAFORM);
		
		switch (cur_mode) {
			case INSPECT:
				cam_track.imgui();
			break;
			case BUILD:
				ImGui::Checkbox("Toggle Traffic Lights", &toggle_traffic_light);
			break;
			case BULLDOZE:
				
			break;
			case TERRAFORM:
				terraform.imgui(heightmap);
			break;
		}

		ImGui::PopID();
	}

	void update (Heightmap& heightmap, Entities& entities, Network& net, View3D& view, Input& input) {
		switch (cur_mode) {
			case INSPECT:
				update_inspect(entities, net, view, input);
			break;
			case BUILD:
				update_build(entities, net, view, input);
			break;
			case BULLDOZE:
				update_bulldoze();
			break;
			case TERRAFORM:
				terraform.update(heightmap, view, input);
			break;
		}

		highlight_hover_sel();
	}

	void update_inspect (Entities& entities, Network& net, View3D& view, Input& input) {
		find_hover(entities, net, view, input, false);

		if (input.buttons[MOUSE_BUTTON_LEFT].went_down) {
			selection = hover;
		}
	}

	void update_build (Entities& entities, Network& net, View3D& view, Input& input) {
		selection = nullptr;

		if (toggle_traffic_light) {
			find_hover(entities, net, view, input, true);
			
			if (input.buttons[MOUSE_BUTTON_LEFT].went_down) {
				auto* node = hover.get<network::Node*>();
				if (node) {
					node->toggle_traffic_light();
					entities.buildings_changed = true; // TODO: make more efficient, or refactor at least?
				}
			}
		}
	}

	void update_bulldoze () {
		selection = nullptr;
	}

////

	void find_hover (Entities& entities, Network& net, View3D& view, Input& input,
			bool only_net) { // TODO: create a mask for this
		Ray ray;
		if (!view.cursor_ray(input, &ray.pos, &ray.dir))
			return;

		hover = {};
		float dist = INF;

		if (!only_net) {
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
		}

		for (auto& node : net.nodes) {
			auto shape = node->get_sel_shape();
			float hit_dist;
			if (shape.test(ray, &hit_dist) && hit_dist < dist) {
				hover = node.get();
				dist = hit_dist;
			}
		}
	}
	void highlight_hover_sel () {
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
	}

};
