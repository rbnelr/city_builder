#pragma once
#include "common.hpp"
#include "util.hpp"
#include "entities.hpp"
#include "terrain.hpp"
#include "game_camera.hpp"

class App;
class Interaction;

class ITool {
public:
	virtual ~ITool () {};

	virtual const char* name () = 0;
	virtual void imgui (App& app) {}

	virtual void select (Interaction& inter, App& app) {} // on tool becomes active
	virtual void deselect (Interaction& inter, App& app) {} // on tool beceomes inactive

	virtual void update (Interaction& inter, App& app, View3D& view) = 0;
};

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

	std::vector<std::unique_ptr<ITool>> tools;
	ITool* cur_tool = nullptr;

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

	auto switch_to_tool (App& app, ITool* tool);
};
