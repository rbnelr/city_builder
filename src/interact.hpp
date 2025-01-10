#pragma once
#include "common.hpp"
#include "util.hpp"
#include "game_camera.hpp"

class App;
class Interaction;
class ToolshelfTool;
class Heightmap;
class Network;
class Entities;

// Basic toggle tool, keeps its own active state and users receive (de)activation and update events
class ToggleTool {
friend class ToolshelfTool;
friend class ExclusiveTool;
	std::string name;
	ToolshelfTool* parent = nullptr;

	bool active = false;
public:

	bool is_active () { return active; }

	virtual ~ToggleTool () {}
	ToggleTool (std::string name): name{std::move(name)} {}

	void toggle (Interaction& I) {
		if (active) deactivate(I);
		else        activate(I);
	}

	//void draw () handled in toolshelf

	virtual void activate (Interaction& I) {
		if (!active) {
			active = true;
			on_activate(I);
		}
	}
	virtual void deactivate (Interaction& I) {
		if (active) {
			active = false;
			on_deactivate(I);
		}
	}
	
	virtual void on_activate (Interaction& I) {};
	virtual void on_deactivate (Interaction& I) {};
	
	// show options for tool
	// recursively called in ToolshelfTool::imgui() on any active children
	virtual void imgui (Interaction& I) {}
	
	// recursively called in ToolshelfTool::update() on any active children
	// NOTE: must be overriden in some custom toolshelfs if they need to pass along custom data when updating their children
	virtual void update (Interaction& I) {};
};
// The usual tool that can be toggled, but when activated, automatically deactivates other siblings in toolshelf (enforcing only one active ExclusiveTool)
class ExclusiveTool : public ToggleTool {
public:
	virtual void activate (Interaction& I) override;
	virtual ~ExclusiveTool () {};
	ExclusiveTool (std::string name): ToggleTool{std::move(name)} {}
};
// Toolshelf that allows nesting of tools and automatically deactivaes child tools of deactivated
class ToolshelfTool : public ExclusiveTool {
public:
	std::vector<std::unique_ptr<ToggleTool>> children;

	virtual ~ToolshelfTool () {};
	ToolshelfTool (std::string name): ExclusiveTool{std::move(name)} {}

	void add_tool (std::unique_ptr<ToggleTool>&& tool) {
		tool->parent = this;
		children.push_back(std::move(tool));
	}
	
	virtual void deactivate (Interaction& I) override {
		// recursively deactivate chilren first
		for (auto& tool : children) {
			tool->deactivate(I);
		}
		// now deactivate us
		ToggleTool::deactivate(I);
	}
	
	// only call on root object
	static void draw_tree (Interaction& I, ToolshelfTool* root_toolshelf) {
		// somehow draw things in breadth-first order (draw level 1 first, then level 2 etc)

		// breath first drawing
		std::queue<ToggleTool*> queue;
		auto enqueue_draw_children = [&queue] (ToolshelfTool* toolshelf) {
			if (toolshelf->active) {
				for (auto& child : toolshelf->children) {
					queue.push(child.get());
				}
			}
		};

		root_toolshelf->active = true; // root toolshelf can't be inactive
		enqueue_draw_children(root_toolshelf);

		while (!queue.empty()) {
			int tools_in_level = (int)queue.size();

			// draw current level tools
			for (int i=0; i<tools_in_level; ++i) {
				ToggleTool* tool = queue.front();
				queue.pop();
				
				{ // Auto wrap horizontal buttons
					float item_width = ImGui::CalcTextSize(tool->name.c_str()).x + ImGui::GetStyle().FramePadding.x * 2;

					ImGui::SameLine(); // needed so GetCursorPosX() actually indicates the horizontal position of the nex item
					// then can put the first and any item that goes off screen onto a new line again
					if (tool == tool->parent->children[0].get() || ImGui::GetCursorPosX() + item_width > ImGui::GetWindowWidth())
						ImGui::NewLine();
				}

				// TODO: currently tools are toggled while children being enqueued, not sure if this is a good idea
				// might build the queue first, then process toggled tools? not sure how to best do that if using imgui
				{
					ImGui::PushID(tool);
					
					auto str = prints(tool->active ? "[%s]###":"%s###", tool->name.c_str());
					if (ImGui::ButtonEx(str.c_str(), ImVec2(0,20), ImGuiButtonFlags_PressedOnClick)) {
						tool->toggle(I);
					}

					ImGui::PopID();
				}
				
				if (auto* toolshelf = dynamic_cast<ToolshelfTool*>(tool)) {
					enqueue_draw_children(toolshelf);
				}
			}

			// current level done
			ImGui::NewLine();
			ImGui::Separator();
		}
	}
	
	void imgui (Interaction& I) override {
		for (auto& tool : children) {
			if (tool->active) {
				ImGui::SeparatorText(concat(tool->name, " Options").c_str());
				tool->imgui(I);
			}
		}
	}
	
	void update (Interaction& I) override {
		for (auto& tool : children) {
			if (tool->active) {
				tool->update(I);
			}
		}
	}
};

inline void ExclusiveTool::activate (Interaction& I) {
	// deactivate siblings first
	if (parent) {
		for (auto& sibling : parent->children) {
			sibling->deactivate(I);
		}
	}
	// now activate us
	ToggleTool::activate(I);
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

	ToolshelfTool root_tool = ToolshelfTool("<root>");

	void _add_tools ();

	template <typename T>
	void clear_sel () {
		if (hover.get<T>())
			hover = nullptr;
		if (selection.get<T>())
			selection = nullptr;
	}

	void imgui ();

	void update (App& app, View3D& view);
	
	Interaction (Input& input, View3D& view, Assets& assets, Heightmap& heightmap, Network& network, Entities& entities):
	             input{input}, view{view}, assets{assets}, heightmap{heightmap}, network{network}, entities{entities} {
		_add_tools();
	}

// dependencies used in most tools
	Input& input;
	View3D& view;

	Assets& assets;
	Heightmap& heightmap;
	Network& network;
	Entities& entities;
	
// Raycasting
	// TODO: create a mask for this
	void find_hover (bool only_net);

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
