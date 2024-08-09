#pragma once
#include "common.hpp"
#include "network.hpp"
#include "entities.hpp"

typedef NullableVariant<Person*, network::Node*> sel_ptr;

inline constexpr const char* NetBuildMode_str[] = {
	"None", "Traffic Lights"
};

struct Interaction {
	enum Mode {
		INSPECT=0,
		BUILD,
		BULLDOZE
	};

	Mode cur_mode = INSPECT;
	
	sel_ptr hover;
	sel_ptr selection;

	template <typename T>
	void clear_sel () {
		if (hover.get<T>())
			hover = nullptr;
		if (selection.get<T>())
			selection = nullptr;
	}

	void imgui () {
		if (!imgui_Header("Interaction")) return;

		auto mode_check = [&] (Mode mode) {
			bool active = mode == cur_mode;
			if (ImGui::ButtonEx(NetBuildMode_str[(int)mode], ImVec2()))
				cur_mode = mode;
		};
		mode_check(INSPECT);
		ImGui::SameLine();
		mode_check(BUILD);
		ImGui::SameLine();
		mode_check(BULLDOZE);

		ImGui::PopID();
	}

	void update (Network& net) {
		switch (cur_mode) {
			case INSPECT:
				update_inspect();
			break;
			case BUILD:
				update_build();
			break;
			case BULLDOZE:
				update_bulldoze();
			break;
		}
	}

	void update_inspect () {

	}

	void update_build () {

	}

	void update_bulldoze () {

	}

////

	void update_selection (Entities& entities, Network& net, View3D const& view, Input& input) {
		Ray ray;
		if (!view.cursor_ray(input, &ray.pos, &ray.dir))
			return;

		hover = {};
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
			selection = hover;
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
	}

};
