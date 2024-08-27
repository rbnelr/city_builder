#pragma once
#include "common.hpp"
#include "engine/opengl.hpp"
#include "app.hpp"
#include "agnostic_render.hpp"
#include "gl_dbgdraw.hpp"
#include "textures.hpp"
#include "assets.hpp"

namespace ogl {

static constexpr int UBO_BINDING_COMMON = 0;
static constexpr int SSBO_BINDING_BINDLESS_TEX_LUT = 0;
static constexpr int SSBO_BINDING_DBGDRAW_INDIRECT = 1;
static constexpr int SSBO_BINDING_ENTITY_INSTANCES = 2;
static constexpr int SSBO_BINDING_ENTITY_MDI       = 3;
static constexpr int SSBO_BINDING_ENTITY_MESH_INFO = 4;
static constexpr int SSBO_BINDING_ENTITY_LOD_INFO  = 5;

struct Exposure {
	SERIALIZE(Exposure, exposure)
	float exposure = 1.0f;
	
	void imgui () {
		ImGui::Text("For how many Lux is the camera exposure set?");
		// Note: slider is backwards because lux is the useful unit, but 'more' exposure means exposing for less lux
		ImGui::SliderFloat("exposure", &exposure, 100000.0f, 0.0001f, "%12.4f", ImGuiSliderFlags_Logarithmic);
	}
};

struct Lighting {
	SERIALIZE(Lighting, sun_col, sky_col, fog_col, fog_base, fog_falloff)
	
	float exposure;
	float inv_exposure;

	float time_of_day = 0.6f;
	float _pad0;
		
	float4x4 sun2world;
	float4x4 world2sun;
	float4x4 moon2world;
	float4x4 world2moon;
	float4x4 solar2world;
	float4x4 world2solar;

	lrgb sun_col = lrgb(1.0f, 0.95f, 0.8f);
	float _pad2;
	lrgb sky_col = srgb(210, 230, 255);
	float _pad3;
	lrgb skybox_bottom_col = srgb(40,50,60);
	float _pad4;
		
	lrgb fog_col = srgb(210, 230, 255);
	//float _pad3;

	float fog_base    = 0.00005f;
	float fog_falloff = 0.0005f;

	float clouds_z  = 5000.0;								// cloud height in m
	float clouds_sz = 1024.0 * 64.0;						// cloud texture size in m
	float2 clouds_offset = 0;								// cloud uv offset for movement (wraps in [0,1))
	// 15m/s is realistic but seems very slow visually
	float2 clouds_vel = rotate2(deg(30)) * float2(100.0);	// current cloud velocity in m/s

	void update (App& app, Exposure& expo, GameTime::SkyConfig& sky) {
		exposure     = 1.0f / (expo.exposure / 1000.0f );
		inv_exposure =         expo.exposure / 1000.0f;

		time_of_day = app.time.time_of_day;

		sun2world   = (float4x4)sky.sun2world  ;
		world2sun   = (float4x4)sky.world2sun  ;
		moon2world  = (float4x4)sky.moon2world ;
		world2moon  = (float4x4)sky.world2moon ;
		solar2world = (float4x4)sky.solar2world;
		world2solar = (float4x4)sky.world2solar;
			
		// TODO: move this to app?
		clouds_offset += (1.0f / clouds_sz) * clouds_vel * app.sim_dt();
		clouds_offset = wrap(clouds_offset, 1.0f);
	}

	void imgui () {
		if (imgui_Header("Lighting")) {

			imgui_ColorEdit("sun_col", &sun_col);
			imgui_ColorEdit("sky_col", &sky_col);
			imgui_ColorEdit("fog_col", &fog_col);

			ImGui::DragFloat("fog_base",    &fog_base,    0.0000001f, 0,0, "%.10f");
			ImGui::DragFloat("fog_falloff", &fog_falloff, 0.000001f, 0,0, "%.10f");

			ImGui::PopID();
		}
	}
};

struct CommonUniforms {
	Ubo ubo = {"common_ubo"};

	struct Common {
		View3D view;
		Lighting lighting;
	};

	void begin () {
		glBindBuffer(GL_UNIFORM_BUFFER, ubo);
		glBufferData(GL_UNIFORM_BUFFER, sizeof(Common), nullptr, GL_STREAM_DRAW);
		glBindBufferBase(GL_UNIFORM_BUFFER, UBO_BINDING_COMMON, ubo);
		glBindBuffer(GL_UNIFORM_BUFFER, 0);
	}
	void upload (size_t offset, size_t size, void const* data) {
		glBindBuffer(GL_UNIFORM_BUFFER, ubo);
		glBufferSubData(GL_UNIFORM_BUFFER, offset, size, data);
		glBindBuffer(GL_UNIFORM_BUFFER, 0);
	}
	void set_lighting (Lighting const& l) {
		upload(offsetof(Common, lighting), sizeof(l), &l);
	}
	void set_view (View3D const& view) {
		upload(offsetof(Common, view), sizeof(view), &view);
	}
};

} // namespace ogl
