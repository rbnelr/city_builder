#pragma once

//// Config
#define RENDERER_DEBUG_LABELS 1

#define OGL_USE_REVERSE_DEPTH 1
#define OGL_USE_DEDICATED_GPU 1
#define RENDERER_WINDOW_FBO_NO_DEPTH 0 // 1 if all 3d rendering happens in an FBO anyway (usually with HDR via float)

//
#if   BUILD_DEBUG

#define RENDERER_PROFILING					1
#define RENDERER_DEBUG_OUTPUT				1
#define RENDERER_DEBUG_OUTPUT_BREAKPOINT	1
#define OGL_STATE_ASSERT					1

#elif BUILD_VALIDATE

#define RENDERER_PROFILING					1
#define RENDERER_DEBUG_OUTPUT				1
#define RENDERER_DEBUG_OUTPUT_BREAKPOINT	0
#define OGL_STATE_ASSERT					0

#elif BUILD_TRACY

//#define NDEBUG // no asserts

#define RENDERER_PROFILING					1 // Could impact perf? Maybe disable this?

#elif BUILD_RELEASE

//#define NDEBUG // no asserts

#endif

/*
	NOTE:
	Not using PCH here due to excessive size of MSVC ipch files
*/

//// Includes
#include "stdint.h"
#include "assert.h"
#include "stdio.h"

#include <memory>
#include <cmath>
#include <string>
#include <string_view>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>

#include <variant>
#include <optional>


// kisslib
#include "engine/kisslib/kissmath.hpp"
#include "engine/kisslib/kissmath_colors.hpp"

#include "engine/kisslib/string.hpp"
#include "engine/kisslib/file_io.hpp"
#include "engine/kisslib/macros.hpp"
#include "engine/kisslib/random.hpp"
#include "engine/kisslib/raw_array.hpp"
#include "engine/kisslib/image.hpp"
#include "engine/kisslib/read_directory.hpp"
#include "engine/kisslib/running_average.hpp"
#include "engine/kisslib/stl_extensions.hpp"
#include "engine/kisslib/threadpool.hpp"
#include "engine/kisslib/threadsafe_queue.hpp"
#include "engine/kisslib/timer.hpp"
//#include "engine/kisslib/animation.hpp"
#include "engine/kisslib/collision.hpp"
//#include "engine/kisslib/allocator.hpp"
//#include "engine/kisslib/containers.hpp"
#include "engine/kisslib/serialization.hpp"
using namespace kiss;

#include "tracy/Tracy.hpp"
#include "dear_imgui.hpp"
#include "engine.hpp"
#include "engine/agnostic_render.hpp"

inline render::DebugDraw g_dbgdraw; // really need something like this to be global

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
