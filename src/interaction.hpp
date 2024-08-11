#pragma once
#include "common.hpp"
#include "network.hpp"
#include "entities.hpp"

typedef NullableVariant<Person*, network::Node*> sel_ptr;

struct Interaction {
	enum Mode {
		INSPECT=0,
		BUILD,
		BULLDOZE
	};
	static inline constexpr const char* Mode_str[] = {
		"Inspect", "Build", "Bulldoze"
	};

	Mode cur_mode = INSPECT;
	
	sel_ptr hover;
	sel_ptr selection;
	
	bool toggle_traffic_light = false;

	template <typename T>
	void clear_sel () {
		if (hover.get<T>())
			hover = nullptr;
		if (selection.get<T>())
			selection = nullptr;
	}

	void imgui () {
		if (!imgui_Header("Interaction", true)) return;

		auto mode_check = [&] (Mode mode) {
			bool active = mode == cur_mode;
			//ImGui::Setstyle
			auto str = prints(active ? "[%s]###%s":"%s###%s", Mode_str[(int)mode], Mode_str[(int)mode]);
			if (ImGui::ButtonEx(str.c_str(), ImVec2(80, 20), ImGuiButtonFlags_PressedOnClick))
				cur_mode = mode;
		};
		mode_check(INSPECT);
		ImGui::SameLine();
		mode_check(BUILD);
		ImGui::SameLine();
		mode_check(BULLDOZE);
		
		if (cur_mode == BUILD) {
			ImGui::Checkbox("Toggle Traffic Lights", &toggle_traffic_light);
		}

		ImGui::PopID();
	}

	void update (Entities& entities, Network& net, View3D const& view, Input& input) {
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
		}

		highlight_hover_sel();
	}

	void update_inspect (Entities& entities, Network& net, View3D const& view, Input& input) {
		find_hover(entities, net, view, input, false);

		if (input.buttons[MOUSE_BUTTON_LEFT].went_down) {
			selection = hover;
		}
	}

	void update_build (Entities& entities, Network& net, View3D const& view, Input& input) {
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

	void find_hover (Entities& entities, Network& net, View3D const& view, Input& input,
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
