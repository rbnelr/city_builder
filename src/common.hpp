#pragma once

//// Config
#define RENDERER_DEBUG_LABELS 1

#define OGL_USE_REVERSE_DEPTH 0
#define OGL_USE_DEDICATED_GPU 1
#define RENDERER_WINDOW_FBO_NO_DEPTH 1 // 1 if all 3d rendering happens in an FBO anyway (usually with HDR via float)

//
#if   BUILD_DEBUG

#define RENDERER_PROFILING					1
#define RENDERER_DEBUG_OUTPUT				1
#define RENDERER_DEBUG_OUTPUT_BREAKPOINT	1
#define OGL_STATE_ASSERT					1
#define IMGUI_DEMO							1

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

#include <cmath>
#include <string>
#include <string_view>
#include <vector>
#include <unordered_map>
#include <memory>

// kisslib
#include "engine/kisslib/kissmath.hpp"
#include "engine/kisslib/kissmath_colors.hpp"

#include "engine/kisslib/string.hpp"
#include "engine/kisslib/file_io.hpp"
#include "engine/kisslib/circular_buffer.hpp"
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
#include "engine/generic_render.hpp"
