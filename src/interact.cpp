#include "common.hpp"
#include "interact.hpp"
#include "app.hpp"
#include "terrain.hpp"
#include "entities.hpp"
#include "network.hpp"
#include "network_sim.hpp"

// TODO: What's the best way to implement it such that "inpecting" is just what happens while not using any real tool?
// Inpecting currently just means being able to select anything (which causes imgui options to show)
// Usually using any other tools should deselect inspected entities, and tools might keep their own selections
class InspectTool : public ExclusiveTool {
	bool dbg_repath = false;
public:
	InspectTool (): ExclusiveTool{"Inspect"} {}

	void imgui (Interaction& I) override {
		ImGui::Checkbox("dbg_repath", &dbg_repath);

		I.cam_track.imgui();
	}

	void update (Interaction& I) override {
		I.find_hover(false);

		// TODO: move this somewhere else? ie. make this an function of vehicle?
		// currently it's just a debug feature though
		if (dbg_repath && I.selection.get<Vehicle*>()) {
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
		
		if (I.input.buttons[MOUSE_BUTTON_LEFT].went_down) {
			I.selection = I.hover;
		}
	}
	
	void on_deactivate (Interaction& I) override {
		I.selection = nullptr;
	}
};

class BuildTools : public ToolshelfTool {
	class ToggleTrafficLights : public ExclusiveTool {
	public:
		ToggleTrafficLights (): ExclusiveTool{"Toggle Traffic Lights"} {}

		void update (Interaction& I) override {
			I.find_hover(true);
			
			if (I.input.buttons[MOUSE_BUTTON_LEFT].went_down) {
				auto* node = I.hover.get<network::Node*>();
				if (node) {
					node->toggle_traffic_light();
					I.entities.buildings_changed = true; // TODO: make more efficient, or refactor at least?
				}
			}
		}
	};

public:
	BuildTools (): ToolshelfTool{"Build"} {
		add_tool(std::make_unique<ToggleTrafficLights>());
	}
};

class BulldozeTool : public ExclusiveTool {
public:
	BulldozeTool (): ExclusiveTool{"Bulldoze"} {}

	void update (Interaction& I) override {
		I.find_hover(false);
		
		if (I.input.buttons[MOUSE_BUTTON_LEFT].went_down) {
			Interaction::remove_entity(I.hover);
		}
	}
};

//class TerraformTools : public ToolshelfTool {
//	HeightmapTerraform terraform;
//public:
//	const char* name () override { return "Terraform"; }
//	void imgui (App& app) override {
//		terraform.imgui(app.heightmap);
//	}
//
//	void update (Interaction& inter, App& app, View3D& view) override {
//		terraform.update(view, app.input, app.heightmap);
//		app.input.buttons[MOUSE_BUTTON_RIGHT].went_down = false; // always eat RMB
//	}
//};

void Interaction::_add_tools () {
	root_tool.add_tool(std::make_unique<InspectTool>());
	root_tool.add_tool(std::make_unique<BuildTools>());
	root_tool.add_tool(std::make_unique<BulldozeTool>());
	root_tool.add_tool(std::make_unique<network::DebugVehicles>());
	//tools.push_back(std::make_unique<TerraformTool>());
}

void Interaction::remove_entity (sel_ptr& entity) {
	auto* veh = entity.get<Vehicle*>();
	if (veh) {
		veh->owner->remove_vehicle(veh);
		entity = nullptr;
	}
}

void Interaction::imgui () {
	if (!imgui_Header("Interaction", true)) return;
	
	ToolshelfTool::draw_tree(*this, &root_tool);

	root_tool.imgui(*this);

	ImGui::PopID();
}

void Interaction::update (App& app, View3D& view) {
	ZoneScoped;
	
	hover = nullptr;
	//hover_pos = {};
	
	root_tool.update(*this);
	
	//// ESC or RMB deselects current tool (=> Go to inspect tool)
	//if (app.input.buttons[KEY_ESCAPE].went_down || app.input.buttons[MOUSE_BUTTON_RIGHT].went_down) {
	//	switch_to_tool(app, tools[0].get());
	//}

	if (app.input.buttons[KEY_DELETE].went_down) {
		remove_entity(selection);
	}

	highlight_hover_sel();
}

void Interaction::find_hover (bool only_net) { // TODO: create a mask for this
	ZoneScoped;

	Ray ray;
	if (!view.cursor_ray(input, &ray.pos, &ray.dir))
		return;
	
	if (input.buttons[KEY_R].is_down) {
		hover_pos.rot += input.mouse_delta.x * deg(180) / 500.0f;
		
		//hover_pos.rot += app.input.mouse_wheel_delta * deg(45); // mouse wheel still zoom camera
	}
	else {
		// lock pos in place while dragging for rotation
		hover_pos.pos = heightmap.raycast_cursor(view, input);
	}

	hover = nullptr;
	float dist = INF;

	if (!only_net) {
		for (auto& person : entities.persons) {
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

		for (auto& veh : network.debug_vehicles.vehicles) {
			if (veh->selectable()) {
				auto shape = veh->get_sel_shape();
				float hit_dist;
				if (shape && shape->test(ray, &hit_dist) && hit_dist < dist) {
					hover = veh.get();
					dist = hit_dist;
				}
			}
		}

		for (auto& building : entities.buildings) {
			auto shape = building->get_sel_shape();
			float hit_dist;
			if (shape && shape->test(ray, &hit_dist) && hit_dist < dist) {
				hover = building.get();
				dist = hit_dist;
			}
		}
	}

	for (auto& node : network.nodes) {
		auto shape = node->get_sel_shape();
		float hit_dist;
		if (shape && shape->test(ray, &hit_dist) && hit_dist < dist) {
			hover = node.get();
			dist = hit_dist;
		}
	}

	for (auto& seg : network.segments) {
		auto shape = seg->get_sel_shape();
		float hit_dist;
		if (shape && shape->test(ray, &hit_dist) && hit_dist < dist) {
			hover = seg.get();
			dist = hit_dist;
		}
	}
}
