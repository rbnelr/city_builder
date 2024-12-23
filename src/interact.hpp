#pragma once
#include "common.hpp"
#include "util.hpp"
#include "entities.hpp"
#include "terrain.hpp"
#include "game_camera.hpp"

class App;
class Interaction;

class ToolshelfTool;

class ToggleTool {
public:
	std::string name;
	ToolshelfTool* parent = nullptr;

	bool active = false;

	virtual ~ToggleTool () {}

	void toggle (App& app) {
		if (active) deactivate(app);
		else        activate(app);
	}

	void draw (App& app) {
		ImGui::PushID(this);
		
		auto str = prints(active ? "[%s]###":"%s###", name.c_str());
		if (ImGui::ButtonEx(str.c_str(), ImVec2(80, 20), ImGuiButtonFlags_PressedOnClick)) {
			toggle(app);
		}

		ImGui::PopID();
	}

	virtual void imgui (App& app) {} // show options for tool

	virtual void activate (App& app) {
		if (!active) {
			active = true;
			on_activate(app);
		}
	}
	virtual void deactivate (App& app) {
		if (active) {
			active = false;
			on_deactivate(app);
		}
	}
	
	virtual void on_activate (App& app) {};
	virtual void on_deactivate (App& app) {};
	virtual void on_update (App& app) {};
};
class ExclusiveTool : public ToggleTool { // this could
	virtual void activate (App& app) override; just be a boolean in ToggleTool
public:
	virtual ~ExclusiveTool () {};

};
class ToolshelfTool : public ExclusiveTool {
public:
	std::vector<std::unique_ptr<ToggleTool>> children;

	virtual ~ToolshelfTool () {};
	
	virtual void deactivate (App& app) override {
		// recursively deactivate chilren first
		for (auto& tool : children) {
			tool->deactivate(app);
		}
		// now deactivate us
		ToggleTool::deactivate(app);
	}
	
	void draw_tree (App& app) {
		// somehow draw things in breath-first order (draw level 1 first, then level 2 etc)

		// breath first drawing
		std::queue<ToggleTool*> queue;
		auto enqueue_draw_children = [&queue] (ToolshelfTool* toolshelf) {
			if (toolshelf->active) {
				for (auto& child : toolshelf->children) {
					queue.push(child.get());
				}
			}
		};

		enqueue_draw_children(this);

		while (!queue.empty()) {
			int tools_in_level = queue.size();

			// draw current level tools
			for (int i=0; i<tools_in_level; ++i) {
				ToggleTool* tool = queue.front();
				queue.pop();

				// Auto wrap horizontal buttons
				ImGui::SameLine();
				if (tool == tool->parent->children[0].get() || ImGui::GetCursorPosX() > ImGui::GetWindowWidth() - 100)
					ImGui::NewLine();
				
				tool->draw(app);
				
				if (auto* toolshelf = dynamic_cast<ToolshelfTool*>(tool)) {
					enqueue_draw_children(toolshelf);
				}
			}

			// current level done
			ImGui::NewLine();
			ImGui::Separator();
		}
	}
	
	// call imgui() on active tools in depth-first order
	void imgui_tree (App& app) {
		for (auto& tool : children) {
			if (tool->active) {
				ImGui::SeparatorText(concat(tool->name, " Options").c_str());
				tool->imgui(app);
			
				if (auto* toolshelf = dynamic_cast<ToolshelfTool*>(tool.get())) {
					toolshelf->update_tree(app);
				}
			}
		}
	}

	// call on_update() on active tools in depth-first order
	void update_tree (App& app) {
		for (auto& tool : children) {
			if (tool->active) {
				tool->on_update(app);
			
				if (auto* toolshelf = dynamic_cast<ToolshelfTool*>(tool.get())) {
					toolshelf->update_tree(app);
				}
			}
		}
	}
};

inline void ExclusiveTool::activate (App& app) {
	// deactivate siblings first
	if (parent) {
		for (auto& sibling : parent->children) {
			sibling->deactivate(app);
		}
	}
	// now activate us
	ToggleTool::activate(app);
}

// TODO: rework this!, probably mirror how heightmap with its tools works
class Interaction {
public:
	// TODO: Does it make sense to keep hover and selection here?

	sel_ptr hover;
	sel_ptr selection;

	struct HoverPos {
		std::optional<float3> pos;
		float rot = 0;

		std::optional<PosRot> get () const {
			if (!pos) return {};
			return PosRot{ *pos, rot };
		}
	};
	HoverPos hover_pos;

	CameraTrack cam_track;

	ToolshelfTool root_tool;

	Interaction ();

	template <typename T>
	void clear_sel () {
		if (hover.get<T>())
			hover = nullptr;
		if (selection.get<T>())
			selection = nullptr;
	}

	void imgui (App& app);

	void update (App& app, View3D& view);

////
	// TODO: create a mask for this
	void find_hover (App& app, View3D& view, bool only_net);
	void highlight_hover_sel () {
		if (hover) {
			hover.visit([&] (auto& x) {
				auto shape = x->get_sel_shape();
				if (shape) shape->highlight(); // does this make sense?
			});
		}
		if (selection) {
			selection.visit([&] (auto& x) {
				auto shape = x->get_sel_shape();
				if (shape) shape->highlight_selected();
			});
		}
	}

	static void remove_entity (sel_ptr& entity);
};
