#include "common.hpp"
#include "ogl_render.hpp"
#include "terrain_render.hpp"
#include "objects.hpp"
#include "render_passes.hpp"
#include "engine/text_render.hpp"

namespace ogl {

class OglRenderer : public Renderer {
	SERIALIZE(OglRenderer, lighting, passes, objects)
public:
	
	virtual void to_json (nlohmann::ordered_json& j) {
		j["ogl_renderer"] = *this;
	}
	virtual void from_json (nlohmann::ordered_json const& j) {
		if (j.contains("ogl_renderer")) j.at("ogl_renderer").get_to(*this);
	}
	
	virtual void imgui (App& app) {
		//if (imgui_Header("Renderer", true)) {
		if (ImGui::Begin("Renderer")) {

			passes.exposure.imgui();
			
			passes.imgui();
			textures.imgui();

			objects.imgui();

		#if OGL_USE_REVERSE_DEPTH
			ImGui::Checkbox("reverse_depth", &ogl::reverse_depth);
		#endif

			gl_dbgdraw.imgui(g_dbgdraw.text);

			lod.imgui();

			lighting.imgui();
			terrain.imgui();
		}
		ImGui::End();
	}

	StateManager state;
	
	Lighting lighting;
	LOD_Func lod;
	CommonUniforms common_ubo;
	Textures textures;

	glDebugDraw gl_dbgdraw;
	OverlayRender overlay;
	
	RenderPasses passes;

	TerrainRenderer terrain;
	ObjectRender    objects;


	OglRenderer () {
		
	}

	virtual void reload_textures () {
		textures.reload_all();
	}
	
	virtual void begin (App& app) {
		ZoneScoped;
		gl_dbgdraw.gl_text_render.begin(g_dbgdraw.text); // upload text and init data structures to allow text printing
	}
	virtual void end (App& app, View3D& view) {
		ZoneScoped;
		
		auto sky_config = app.time.calc_sky_config(view);
		
		lighting.update(app, passes.exposure.exposure_mult(), sky_config);
		
	#if RENDERER_DEBUG_LABELS
		// Dummy call because first gl event in nsight is always bugged, and by doing this the next OGL_TRACE() actually works
		glBindVertexArray(0);
	#endif
		
		{
			ZoneScopedN("uploads");
			OGL_TRACE("uploads");

			textures.heightmap.update_changes(app.heightmap);

			if (app.assets.assets_reloaded) {
				ZoneScopedN("assets_reloaded");
				objects.entities.upload_meshes(app.assets);
			}

			if (app.entities.buildings_changed) {
				ZoneScopedN("buildings_changed");

				objects.upload_static_instances(textures, app);
			}

			objects.upload_vehicle_instances(textures, app, view);
			objects.update_dynamic_traffic_signals(textures, app.network);
		}

		{
			ZoneScopedN("setup");
			OGL_TRACE("setup");

			{
				//OGL_TRACE("set state defaults");

				state.wireframe          = gl_dbgdraw.wireframe;
				state.wireframe_no_cull  = gl_dbgdraw.wireframe_no_cull;
				state.wireframe_no_blend = gl_dbgdraw.wireframe_no_blend;

				state.set_default();

				//glEnable(GL_LINE_SMOOTH); // This is extremely slow
				glLineWidth(gl_dbgdraw.line_width);
			}

			{
				common_ubo.begin();
				common_ubo.set_lighting(lighting);
				textures.bindless_textures.update_lut(SSBO_BINDING_BINDLESS_TEX_LUT);

				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, SSBO_BINDING_DBGDRAW_INDIRECT, gl_dbgdraw.indirect_vbo);

				gl_dbgdraw.update(app.input);
			}
		}

		passes.update(state, textures, app.input.window_size);

		auto update_view_resolution = [&] (int2 res) {
			
			view.viewport_size = (float2)res;
			view.inv_viewport_size = 1.0f / view.viewport_size;

			common_ubo.set_view(view);
		};
		

		// Lod & Cull with real camera if in dbg camera and dbg_lodcull > 0
		View3D real_view = app.dbg_lodcull > 0 && app.view_dbg_cam ? app.dbg_get_main_view() : view;
		View3D shadowmap_casc_cull;

		if (passes.shadowmap) {
			ZoneScopedN("shadow_pass");
			OGL_TRACE("shadow_pass");

			int counter = 0;
			passes.shadowmap->draw_cascades(real_view, sky_config, state, [&] (View3D& shadow_view, TextureView& depth_tex) {
				common_ubo.set_view(shadow_view);
				
				if (++counter + 1 == app.dbg_lodcull)
					shadowmap_casc_cull = shadow_view;

				terrain.render_terrain(state, app.heightmap, textures, real_view, shadow_view, true);
				objects.clippings.render(state, depth_tex, true);
		
				objects.networks.render(state, textures, true);

				objects.entities.draw_all(state, true);
			});
		}
		
		update_view_resolution(passes.renderscale.size);

		{
			ZoneScopedN("geometry_pass");
			OGL_TRACE("geometry_pass");
			
			passes.begin_geometry_pass(state);
			
			// visualize shadowmap culling with app.dbg_lodcull > 1
			View3D& cull_view = app.dbg_lodcull > 1 ? shadowmap_casc_cull : real_view;
			if (app.dbg_lodcull > 0) {
				dbgdraw_frustrum(cull_view.clip2world, lrgba(0,1,0,1));
			}

			terrain.render_terrain(state, app.heightmap, textures, real_view, cull_view);
			objects.clippings.render(state, passes.gbuf.depth);

			objects.networks.render(state, textures);
			objects.decals.render(state, passes.gbuf, textures);
			objects.curved_decals.render(state, passes.gbuf);

			overlay.render(state, passes.gbuf, app, textures);

			objects.entities.draw_all(state);
		}

		{
			ZoneScopedN("deferred_lighting");
			OGL_TRACE("deferred_lighting");
			
			passes.deferred_lighting_pass(state, textures, objects.entities.lights);
		}
		
		update_view_resolution(app.input.window_size);

		passes.postprocess(state, app.input.window_size, app.input.real_dt);

		gl_dbgdraw.render(state, g_dbgdraw);

		{
			ZoneScopedN("draw ui");
			OGL_TRACE("draw ui");
		
			if (app.trigger_screenshot && !app.screenshot_hud) take_screenshot(app.input.window_size);
		
			// draw HUD
			app.draw_imgui();

			if (app.trigger_screenshot && app.screenshot_hud)  take_screenshot(app.input.window_size);
			app.trigger_screenshot = false;
		}
	}
};

} // namespace ogl

std::unique_ptr<Renderer> create_ogl_backend () {
	ZoneScoped;
	return std::make_unique<ogl::OglRenderer>();
}
