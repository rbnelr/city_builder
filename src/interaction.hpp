#pragma once
#include "common.hpp"
#include "util.hpp"
#include "network.hpp"
#include "entities.hpp"
#include "terrain.hpp"

// TODO: rework this!, probably mirror how heightmap with its tools works
struct Interaction {
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

	void update (View3D& view, Input& input,
			OverlayDraw& overlay, Entities& entities, Network& net, Heightmap& heightmap) {
		switch (cur_mode) {
			case INSPECT:
				update_inspect(view, input, overlay, entities, net, heightmap);
			break;
			case BUILD:
				update_build(view, input, entities, net);
			break;
			case BULLDOZE:
				update_bulldoze(view, input, entities, net);
			break;
			case TERRAFORM:
				terraform.update(view, input, heightmap);
			break;
		}

		highlight_hover_sel();
	}

	void update_inspect (View3D& view, Input& input,
			OverlayDraw& overlay, Entities& entities, Network& net, Heightmap& heightmap) {
		find_hover(view, input, entities, net, false);

		if (selection.get<Person*>()) {
			auto* pers = selection.get<Person*>();

			if (pers->trip) {
				network::VehNavPoint targ = {};

				auto* build = hover.get<Building*>();
				if (build) {
					targ = network::VehNavPoint::from_building(build);
				}
				else {
					auto res = heightmap.raycast_cursor(view, input);
					if (res) targ = network::VehNavPoint::from_free_point(net, *res);
				}

				if (targ) {
					if (input.buttons[MOUSE_BUTTON_RIGHT].went_down) {
						if (build) {
							pers->trip->target = build; // switch target building
						}
						else {
							// targeting free point, keep building target
							pers->trip->waypoint = targ;
						}
						pers->trip->sim.nav.repath(net, targ);
					}
					else {
						// copy and preview repath
						network::VehNav tmp_nav = pers->trip->sim.nav;
						if (tmp_nav.repath(net, targ)) {
							tmp_nav.visualize(overlay, net, &pers->trip->sim, false, lrgba(1,1,1,0.4f));
						}
					}
				}
			}
		}
		
		if (input.buttons[MOUSE_BUTTON_LEFT].went_down) {
			selection = hover;
		}
	}

	void update_build (View3D& view, Input& input, Entities& entities, Network& net) {
		selection = nullptr;

		if (toggle_traffic_light) {
			find_hover(view, input, entities, net, true);
			
			if (input.buttons[MOUSE_BUTTON_LEFT].went_down) {
				auto* node = hover.get<network::Node*>();
				if (node) {
					node->toggle_traffic_light();
					entities.buildings_changed = true; // TODO: make more efficient, or refactor at least?
				}
			}
		}
	}

	void update_bulldoze (View3D& view, Input& input, Entities& entities, Network& net) {
		selection = nullptr;
		
		find_hover(view, input, entities, net, false);
		
		if (input.buttons[MOUSE_BUTTON_LEFT].went_down) {
			if (hover.get<Person*>()) {
				auto* pers = hover.get<Person*>();
				if (pers->trip) {
					pers->trip->cancel_trip(pers);
					pers->trip = nullptr;
				}

				hover = nullptr;
			}
		}
	}

////

	void find_hover (View3D& view, Input& input, Entities& entities, Network& net,
			bool only_net) { // TODO: create a mask for this
		Ray ray;
		if (!view.cursor_ray(input, &ray.pos, &ray.dir))
			return;

		hover = nullptr;
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

			for (auto& building : entities.buildings) {
				auto shape = building->get_sel_shape();
				float hit_dist;
				if (shape.test(ray, &hit_dist) && hit_dist < dist) {
					hover = building.get();
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

		for (auto& seg : net.segments) {
			auto shape = seg->get_sel_shape();
			float hit_dist;
			if (shape.test(ray, &hit_dist) && hit_dist < dist) {
				hover = seg.get();
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
