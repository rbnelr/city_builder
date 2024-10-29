#include "common.hpp"
#include "interact.hpp"
#include "app.hpp"
#include "network.hpp"
#include "network_sim.hpp"

class Inspect : public ITool {
	bool dbg_repath = false;
public:
	const char* name () override { return "Inspect"; }
	void imgui (App& app) override {
		ImGui::SeparatorText("Inspect");
		ImGui::Checkbox("dbg_repath", &dbg_repath);

		app.interact.cam_track.imgui();
	}

	void update (Interaction& inter, App& app, View3D& view) override {
		inter.find_hover(app, view, false);

		// TODO: move this somewhere else? ie. make this an function of vehicle?
		// currently it's just a debug feature though
		if (dbg_repath && inter.selection.get<Vehicle*>()) {
			//auto* pers = selection.get<Person*>();
			//auto* trip = pers->owned_vehicle->get_trip();
			//if (trip) {
			//	std::optional<network::VehicleTrip::Waypoint> targ = {};
			//
			//	auto* build = hover.get<Building*>();
			//	if (build) {
			//		targ = build;
			//	}
			//	else {
			//		if (hover_pos) targ = *hover_pos;
			//	}
			//	
			//	if (targ) {
			//		bool preview = !input.buttons[MOUSE_BUTTON_RIGHT].went_down;
			//		trip->dbg_waypoint(overlay, net, *targ, preview);
			//	}
			//}
		}
		
		if (app.input.buttons[MOUSE_BUTTON_LEFT].went_down) {
			inter.selection = inter.hover;
		}
	}
	
	void deselect (Interaction& inter, App& app) override {
		inter.selection = nullptr;
	}
};

class Build : public ITool {
	bool toggle_traffic_light = false;
public:
	const char* name () override { return "Build"; }
	void imgui (App& app) override {
		ImGui::SeparatorText("Build");
		ImGui::Checkbox("Toggle Traffic Lights", &toggle_traffic_light);
	}

	void update (Interaction& inter, App& app, View3D& view) override {
		if (toggle_traffic_light) {
			inter.find_hover(app, view, true);
			
			if (app.input.buttons[MOUSE_BUTTON_LEFT].went_down) {
				auto* node = inter.hover.get<network::Node*>();
				if (node) {
					node->toggle_traffic_light();
					app.entities.buildings_changed = true; // TODO: make more efficient, or refactor at least?
				}
			}
		}
	}
};

class Bulldoze : public ITool {
public:
	const char* name () override { return "Bulldoze"; }
	//void imgui () override {}

	void update (Interaction& inter, App& app, View3D& view) override {
		inter.find_hover(app, view, false);
		
		//if (app.input.buttons[MOUSE_BUTTON_LEFT].went_down) {
		//	if (inter.hover.get<Vehicle*>()) {
		//		auto* veh = inter.hover.get<Vehicle*>();
		//		auto* trip = veh->get_trip();
		//		if (trip && veh->owner) { // TODO: ??
		//			network::PersonTrip::cancel_trip(*trip, *veh->owner);
		//		}
		//
		//		inter.hover = nullptr;
		//	}
		//}
	}
};

class DebugVehicles : public ITool {
public:
	const char* name () override { return "Debug Vehicles"; }
	void imgui (App& app) override {
		ImGui::SeparatorText("Debug Vehicles");
		app.network.debug_vehicles.imgui();
	}

	void update (Interaction& inter, App& app, View3D& view) override {
		inter.find_hover(app, view, true);

		app.network.debug_vehicles.update_interact(app.input, app.assets, inter.hover, inter.hover_pos.get());
	}
	
	void deselect (Interaction& inter, App& app) override {
		app.network.debug_vehicles.deselect();
	}
};

class Terraform : public ITool {
	HeightmapTerraform terraform;
public:
	const char* name () override { return "Terraform"; }
	void imgui (App& app) override {
		terraform.imgui(app.heightmap);
	}

	void update (Interaction& inter, App& app, View3D& view) override {
		terraform.update(view, app.input, app.heightmap);
		app.input.buttons[MOUSE_BUTTON_RIGHT].went_down = false; // always eat RMB
	}
};

Interaction::Interaction () {
	tools.push_back(std::make_unique<Inspect>());
	tools.push_back(std::make_unique<Build>());
	tools.push_back(std::make_unique<Bulldoze>());
	tools.push_back(std::make_unique<DebugVehicles>());
	tools.push_back(std::make_unique<Terraform>());

	cur_tool = tools[0].get();
}

auto Interaction::switch_to_tool (App& app, ITool* tool) {
	if (cur_tool != tool) {
		cur_tool->deselect(*this, app);
		cur_tool = tool;
		cur_tool->select(*this, app);
	}
}

void Interaction::imgui (App& app) {
	if (!imgui_Header("Interaction", true)) return;
	

	// TODO: I would prefer a automatic line-wrap layouting here, can Imgui do that or would I implement that myself?
	for (auto& tool : tools) {
		//if (&tool != &tools[0]) ImGui::SameLine();
		
		// Auto wrap horizontal buttons
		ImGui::SameLine();
		if (&tool == &tools[0] || ImGui::GetCursorPosX() > ImGui::GetWindowWidth() - 100)
			ImGui::NewLine();
		
		bool active = cur_tool == tool.get();
		auto str = prints(active ? "[%s]###%s":"%s###%s", tool->name(), tool->name());
		if (ImGui::ButtonEx(str.c_str(), ImVec2(80, 20), ImGuiButtonFlags_PressedOnClick)) {
			switch_to_tool(app, tool.get());
		}
	}
	
	cur_tool->imgui(app);

	ImGui::PopID();
}

void Interaction::update (App& app, View3D& view) {
	ZoneScoped;
	
	hover = nullptr;
	//hover_pos = {};

	cur_tool->update(*this, app, view);
	
	// ESC or RMB deselects current tool (=> Go to inspect tool)
	if (app.input.buttons[KEY_ESCAPE].went_down || app.input.buttons[MOUSE_BUTTON_RIGHT].went_down) {
		switch_to_tool(app, tools[0].get());
	}

	highlight_hover_sel();
}

void Interaction::find_hover (App& app, View3D& view, bool only_net) { // TODO: create a mask for this
	ZoneScoped;

	Ray ray;
	if (!view.cursor_ray(app.input, &ray.pos, &ray.dir))
		return;
	
	if (app.input.buttons[KEY_R].is_down) {
		hover_pos.rot += app.input.mouse_delta.x * deg(180) / 500.0f;
	}
	else {
		// lock pos in place while dragging for rotation
		hover_pos.pos = app.heightmap.raycast_cursor(view, app.input);
	}

	hover = nullptr;
	float dist = INF;

	if (!only_net) {
		for (auto& person : app.entities.persons) {
			auto* veh = person->owned_vehicle.get();
			if (veh->selectable()) {
				auto shape = veh->get_sel_shape();
				float hit_dist;
				if (shape && shape->test(ray, &hit_dist) && hit_dist < dist) {
					hover = veh;
					dist = hit_dist;
				}
			}
		}

		for (auto& veh : app.network.debug_vehicles.vehicles) {
			if (veh->selectable()) {
				auto shape = veh->get_sel_shape();
				float hit_dist;
				if (shape && shape->test(ray, &hit_dist) && hit_dist < dist) {
					hover = veh.get();
					dist = hit_dist;
				}
			}
		}

		for (auto& building : app.entities.buildings) {
			auto shape = building->get_sel_shape();
			float hit_dist;
			if (shape && shape->test(ray, &hit_dist) && hit_dist < dist) {
				hover = building.get();
				dist = hit_dist;
			}
		}
	}

	for (auto& node : app.network.nodes) {
		auto shape = node->get_sel_shape();
		float hit_dist;
		if (shape && shape->test(ray, &hit_dist) && hit_dist < dist) {
			hover = node.get();
			dist = hit_dist;
		}
	}

	for (auto& seg : app.network.segments) {
		auto shape = seg->get_sel_shape();
		float hit_dist;
		if (shape && shape->test(ray, &hit_dist) && hit_dist < dist) {
			hover = seg.get();
			dist = hit_dist;
		}
	}
}
