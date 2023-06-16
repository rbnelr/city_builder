#pragma once
#include "common.hpp"
#include "engine/engine.hpp"

struct App;

struct Renderer {

	virtual ~Renderer () {}
	
	virtual void imgui (App& app) = 0;

	virtual void begin (App& app) = 0;
	virtual void end (App& app) = 0;

	render::DebugDraw dbgdraw;
};

std::unique_ptr<Renderer> create_ogl_backend ();

struct App : public Engine {
	// can't serialize renderer directly, because its abstract
	// with multiple backends we would proably want settings to be inside a normal Renderer class
	// which handles the backend selection and calls the specific backend with the correct data
	SERIALIZE_NONE(App)

	Camera2D cam2d = Camera2D();
	View3D view;

	std::unique_ptr<Renderer> renderer = create_ogl_backend();
	
	App (): Engine{"Kiss-Framework Project"} {}
	virtual ~App () {}

	virtual void json_load () { load("debug.json", this); }
	virtual void json_save () { save("debug.json", *this); }

	virtual void imgui () {
		ZoneScoped;

		renderer->imgui(*this);

		ImGui::Separator();

		if (imgui_Header("2D", true)) {
			cam2d.imgui("cam");
			ImGui::PopID();
		}
	}

	void update () {
		ZoneScoped;

		renderer->dbgdraw.clear();

		view = cam2d.update(input, (float2)input.window_size);

		renderer->dbgdraw.axis_gizmo(view, input.window_size);

		for (int y=0; y<32; ++y)
		for (int x=0; x<32; ++x) {
			renderer->dbgdraw.quad(float3(x + 0.5f, y + 0.5f, 0), 1, lrgba(1,1,1,1));
		}
	}

	virtual void frame () {
		ZoneScoped;
		
		renderer->begin(*this);

		update();

		renderer->end(*this);
	}
};

inline void imgui_style () {
	auto& style = ImGui::GetStyle();
	ImVec4* colors = style.Colors;

	colors[ImGuiCol_Text]                   = ImVec4(0.90f, 0.90f, 0.90f, 1.00f);
	colors[ImGuiCol_TextDisabled]           = ImVec4(0.60f, 0.60f, 0.60f, 1.00f);
	colors[ImGuiCol_WindowBg]               = ImVec4(0.09f, 0.09f, 0.11f, 0.83f);
	colors[ImGuiCol_ChildBg]                = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
	colors[ImGuiCol_PopupBg]                = ImVec4(0.11f, 0.11f, 0.14f, 0.92f);
	colors[ImGuiCol_Border]                 = ImVec4(0.50f, 0.50f, 0.50f, 0.50f);
	colors[ImGuiCol_BorderShadow]           = ImVec4(0.05f, 0.06f, 0.07f, 0.80f);
	colors[ImGuiCol_FrameBg]                = ImVec4(0.43f, 0.43f, 0.43f, 0.39f);
	colors[ImGuiCol_FrameBgHovered]         = ImVec4(0.47f, 0.47f, 0.69f, 0.40f);
	colors[ImGuiCol_FrameBgActive]          = ImVec4(0.42f, 0.41f, 0.64f, 0.69f);
	colors[ImGuiCol_TitleBg]                = ImVec4(0.27f, 0.27f, 0.54f, 0.83f);
	colors[ImGuiCol_TitleBgActive]          = ImVec4(0.32f, 0.32f, 0.63f, 0.87f);
	colors[ImGuiCol_TitleBgCollapsed]       = ImVec4(0.40f, 0.40f, 0.80f, 0.20f);
	colors[ImGuiCol_MenuBarBg]              = ImVec4(0.40f, 0.40f, 0.55f, 0.80f);
	colors[ImGuiCol_ScrollbarBg]            = ImVec4(0.20f, 0.25f, 0.30f, 0.60f);
	colors[ImGuiCol_ScrollbarGrab]          = ImVec4(0.40f, 0.40f, 0.80f, 0.30f);
	colors[ImGuiCol_ScrollbarGrabHovered]   = ImVec4(0.40f, 0.40f, 0.80f, 0.40f);
	colors[ImGuiCol_ScrollbarGrabActive]    = ImVec4(0.41f, 0.39f, 0.80f, 0.60f);
	colors[ImGuiCol_CheckMark]              = ImVec4(0.90f, 0.90f, 0.90f, 0.50f);
	colors[ImGuiCol_SliderGrab]             = ImVec4(1.00f, 1.00f, 1.00f, 0.30f);
	colors[ImGuiCol_SliderGrabActive]       = ImVec4(0.41f, 0.39f, 0.80f, 0.60f);
	colors[ImGuiCol_Button]                 = ImVec4(0.35f, 0.40f, 0.61f, 0.62f);
	colors[ImGuiCol_ButtonHovered]          = ImVec4(0.40f, 0.48f, 0.71f, 0.79f);
	colors[ImGuiCol_ButtonActive]           = ImVec4(0.46f, 0.54f, 0.80f, 1.00f);
	colors[ImGuiCol_Header]                 = ImVec4(0.40f, 0.40f, 0.90f, 0.45f);
	colors[ImGuiCol_HeaderHovered]          = ImVec4(0.45f, 0.45f, 0.90f, 0.80f);
	colors[ImGuiCol_HeaderActive]           = ImVec4(0.53f, 0.53f, 0.87f, 0.80f);
	colors[ImGuiCol_Separator]              = ImVec4(0.50f, 0.50f, 0.50f, 0.60f);
	colors[ImGuiCol_SeparatorHovered]       = ImVec4(0.60f, 0.60f, 0.70f, 1.00f);
	colors[ImGuiCol_SeparatorActive]        = ImVec4(0.70f, 0.70f, 0.90f, 1.00f);
	colors[ImGuiCol_ResizeGrip]             = ImVec4(1.00f, 1.00f, 1.00f, 0.10f);
	colors[ImGuiCol_ResizeGripHovered]      = ImVec4(0.78f, 0.82f, 1.00f, 0.60f);
	colors[ImGuiCol_ResizeGripActive]       = ImVec4(0.78f, 0.82f, 1.00f, 0.90f);
	colors[ImGuiCol_Tab]                    = ImVec4(0.34f, 0.34f, 0.68f, 0.79f);
	colors[ImGuiCol_TabHovered]             = ImVec4(0.45f, 0.45f, 0.90f, 0.80f);
	colors[ImGuiCol_TabActive]              = ImVec4(0.40f, 0.40f, 0.73f, 0.84f);
	colors[ImGuiCol_TabUnfocused]           = ImVec4(0.28f, 0.28f, 0.57f, 0.82f);
	colors[ImGuiCol_TabUnfocusedActive]     = ImVec4(0.35f, 0.35f, 0.65f, 0.84f);
	colors[ImGuiCol_DockingPreview]         = ImVec4(0.40f, 0.40f, 0.90f, 0.31f);
	colors[ImGuiCol_DockingEmptyBg]         = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
	colors[ImGuiCol_PlotLines]              = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
	colors[ImGuiCol_PlotLinesHovered]       = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
	colors[ImGuiCol_PlotHistogram]          = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
	colors[ImGuiCol_PlotHistogramHovered]   = ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
	colors[ImGuiCol_TableHeaderBg]          = ImVec4(0.27f, 0.27f, 0.38f, 1.00f);
	colors[ImGuiCol_TableBorderStrong]      = ImVec4(0.31f, 0.31f, 0.45f, 1.00f);
	colors[ImGuiCol_TableBorderLight]       = ImVec4(0.26f, 0.26f, 0.28f, 1.00f);
	colors[ImGuiCol_TableRowBg]             = ImVec4(0.00f, 0.00f, 0.00f, 0.29f);
	colors[ImGuiCol_TableRowBgAlt]          = ImVec4(0.19f, 0.19f, 0.19f, 0.29f);
	colors[ImGuiCol_TextSelectedBg]         = ImVec4(0.00f, 0.00f, 1.00f, 0.35f);
	colors[ImGuiCol_DragDropTarget]         = ImVec4(1.00f, 1.00f, 0.00f, 0.90f);
	colors[ImGuiCol_NavHighlight]           = ImVec4(0.45f, 0.45f, 0.90f, 0.80f);
	colors[ImGuiCol_NavWindowingHighlight]  = ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
	colors[ImGuiCol_NavWindowingDimBg]      = ImVec4(0.80f, 0.80f, 0.80f, 0.20f);
	colors[ImGuiCol_ModalWindowDimBg]       = ImVec4(0.20f, 0.20f, 0.20f, 0.35f);


	style.WindowPadding     = ImVec2(5,5);
	style.FramePadding      = ImVec2(6,2);
	style.CellPadding       = ImVec2(4,2);
	style.ItemSpacing       = ImVec2(12,3);
	style.ItemInnerSpacing  = ImVec2(3,3);
	style.IndentSpacing     = 18;
	style.GrabMinSize       = 14;

	style.WindowRounding    = 3;
	style.FrameRounding     = 6;
	style.PopupRounding     = 3;
	style.GrabRounding      = 6;

	style.WindowTitleAlign  = ImVec2(0.5f, 0.5f);
}
