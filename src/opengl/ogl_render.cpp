#pragma once
#include "common.hpp"
#include "app.hpp"
#include "engine/opengl.hpp"
#include "agnostic_render.hpp"
#include "gl_dbgdraw.hpp"
#include "render_passes.hpp"

namespace ogl {

#if 0
struct TriRenderer {
	Shader* shad  = g_shaders.compile("tris");

	Texture2D tex = texture2D<srgba8>("logo", "textures/Opengl-logo.png");

	Sampler sampler_normal = sampler("sampler_normal", FILTER_MIPMAPPED, GL_REPEAT, false);

	struct Vertex {
		float3 pos;
		float2 uv;
		float4 col;

		VERTEX_CONFIG(
			ATTRIB(FLT2, Vertex, pos),
			ATTRIB(FLT2, Vertex, uv),
			ATTRIB(FLT4, Vertex, col),
		)
	};

	VertexBufferI vbo_tris = vertex_bufferI<Vertex>("TriRenderer.Vertex");

	std::vector<Vertex>   verticies;
	std::vector<uint16_t> indices;

	void update (Input& I) {
		verticies.clear();
		verticies.shrink_to_fit();
		indices.clear();
		indices.shrink_to_fit();
	}

	void push_quad (float3 pos, float2 size, float4 col) {
		uint16_t idx = (uint16_t)verticies.size();

		auto* pv = push_back(verticies, 4);
		pv[0] = { pos + float3(     0,      0, 0), float2(0,0), col };
		pv[1] = { pos + float3(size.x,      0, 0), float2(1,0), col };
		pv[2] = { pos + float3(size.x, size.y, 0), float2(1,1), col };
		pv[3] = { pos + float3(     0, size.y, 0), float2(0,1), col };

		render::shapes::push_quad_indices<uint16_t>(indices, idx+0u, idx+1u, idx+2u, idx+3u);
	}

	void render (StateManager& state) {
		OGL_TRACE("TriRenderer");

		ZoneScoped;

		if (shad->prog) {
			OGL_TRACE("TriRenderer");

			vbo_tris.stream(verticies, indices);

			if (indices.size() > 0) {
				glUseProgram(shad->prog);

				state.bind_textures(shad, {
					{ "tex", tex, sampler_normal }
				});

				PipelineState s;
				s.depth_test = false;
				s.blend_enable = true;
				state.set(s);

				glBindVertexArray(vbo_tris.vao);
				glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_UNSIGNED_SHORT, (void*)0);
			}
		}

		glBindVertexArray(0);
	}
};
#endif

struct OglRenderer : public Renderer {
	SERIALIZE_NONE(OglRenderer)
	
	virtual void imgui (App& app) {
		if (imgui_Header("Renderer", true)) {
			
			passes.imgui();

		#if OGL_USE_REVERSE_DEPTH
			ImGui::Checkbox("reverse_depth", &ogl::reverse_depth);
		#endif

			gl_dbgdraw.imgui();
			lighting.imgui();
			terrain_renderer.imgui();

			ImGui::PopID();
		}
	}

	StateManager state;
	
	struct Lighting {
		SERIALIZE(Lighting, sun_col, sky_col, fog_col, fog_base, fog_falloff)

		float4 sun_dir;

		lrgb sun_col = lrgb(0.8f, 0.77f, 0.4f) * 2.0f;
		float _pad0;

		lrgb sky_col = srgb(210, 230, 255);
		float _pad1;
		
		lrgb skybox_bottom_col = srgb(40,50,60);
		float _pad2;
		
		lrgb fog_col = srgb(210, 230, 255);
		//float _pad3;

		float fog_base = 0.20f;
		float fog_falloff = 30.0f;

		void imgui () {
			if (imgui_Header("Lighting", true)) {

				imgui_ColorEdit("sun_col", &sun_col);
				imgui_ColorEdit("sky_col", &sky_col);
				imgui_ColorEdit("fog_col", &fog_col);

				ImGui::DragFloat("fog_base/100", &fog_base, 0.001f);
				ImGui::DragFloat("fog_falloff/100", &fog_falloff, 0.01f);

				ImGui::PopID();
			}
		}
	};

	struct CommonUniforms {
		static constexpr int UBO_BINDING = 0;

		Ubo ubo = {"common_ubo"};

		struct Common {
			View3D view;
			Lighting lighting;
		};

		void set (View3D const& view, Lighting& l) {
			Common common = {};
			common.view = view;
			common.lighting = l;
			common.lighting.fog_base = l.fog_base / 100;
			common.lighting.fog_falloff = l.fog_falloff / 100;
			stream_buffer(GL_UNIFORM_BUFFER, ubo, sizeof(common), &common, GL_STREAM_DRAW);

			glBindBuffer(GL_UNIFORM_BUFFER, ubo);
			glBindBufferBase(GL_UNIFORM_BUFFER, UBO_BINDING, ubo);
			glBindBuffer(GL_UNIFORM_BUFFER, 0);
		}
	};
	CommonUniforms common_ubo;
	
	RenderPasses passes;

	Lighting lighting;

	glDebugDraw gl_dbgdraw;

	struct TerrainRenderer {
		
		static constexpr int MAP_SZ = 16*1024;
	
		static constexpr int TERRAIN_CHUNK_SZ = 32; // 128 is max with GL_UNSIGNED_SHORT indices

		Shader* shad_terrain = g_shaders.compile("terrain");
	
		bool draw_terrain = true;
		//bool draw_water = true;

		bool dbg_lod = false;
		
		float lod_offset = 16;
		float lod_fac    = 64;

		int terrain_base_lod = -2;
		int water_base_lod = -1;

		void imgui () {
			if (imgui_Header("TerrainRenderer", true)) {

				ImGui::Checkbox("dbg_lod", &dbg_lod);

				ImGui::Checkbox("draw_terrain", &draw_terrain);
				//ImGui::Checkbox("draw_water", &draw_water);

				ImGui::DragFloat("lod_offset", &lod_offset, 1, 0, MAP_SZ);
				ImGui::DragFloat("lod_fac", &lod_fac, 1, 0, MAP_SZ);

				ImGui::SliderInt("terrain_base_lod", &terrain_base_lod, -6, 6);
				ImGui::SliderInt("water_base_lod", &water_base_lod, -6, 6);

				ImGui::Text("drawn_chunks: %4d  (%7d vertices)", drawn_chunks, drawn_chunks * chunk_vertices);

				ImGui::PopID();
			}
		}

		TerrainRenderer () {
			gen_terrain_quad();
		}

		struct TerrainVertex {
			float2 pos;

			VERTEX_CONFIG(
				ATTRIB(FLT2, TerrainVertex, pos),
			)
		};
		VertexBufferI terrain_chunk = vertex_bufferI<TerrainVertex>("terrain");
		int chunk_vertices;
		int chunk_indices;
		
		int drawn_chunks = 0;

		template <typename FUNC>
		void lodded_chunks (View3D& view, OglRenderer& r, Shader* shad, int base_lod, bool dbg, FUNC chunk) {
			ZoneScoped;

			float2 lod_center = (float2)view.cam_pos;
			// TODO: adjust for terrain height? -> abs distance to full heightmap range might be reasonable
			// -> finding correct "distance" to heightmap terrain is somewhat impossible
			//    there could always be cases where a chunk has too high of a LOD compared to the distance the surface
			//    actually ends up being to the camera due to the heightmap
			float lod_center_z = view.cam_pos.z;

			int2 prev_bound0 = 0;
			int2 prev_bound1 = 0;
			
			if (dbg)
				r.dbgdraw.wire_quad(float3(0), (float2)MAP_SZ, lrgba(0,0,0,1));

			// iterate lods
			for (int lod=base_lod;; lod++) {
				ZoneScopedN("lod");

				// size of this lod version of a chunk
				int sz = lod >= 0 ? TERRAIN_CHUNK_SZ << lod : TERRAIN_CHUNK_SZ >> (-lod);
				if (sz > MAP_SZ)
					break;

				// parent (next higher res) lod chunk size
				int parent_sz = sz << 1;
				// mask that removes floors to this chunk size
				int parent_mask = ~(parent_sz-1);

				float quad_size = (float)sz / (float)TERRAIN_CHUNK_SZ;

				// lod radius formula
				float radius = lod_offset + lod_fac * quad_size;
				if (lod_center_z >= radius)
					continue;
				radius = sqrt(radius*radius - lod_center_z*lod_center_z); // intersection of sphere with <radius> and z=0 plane to apply lodding at z=0 plane

				// for this lod radius, get the chunk grid bounds, aligned such that the parent lod chunks still fit without holes
				int2 bound0 =  floori(lod_center - radius) & parent_mask;
				int2 bound1 = (floori(lod_center + radius) & parent_mask) + parent_sz;
				
				assert(bound1.x > bound0.x && bound1.y > bound0.y); // needed for next check to work (on prev_bound)

				// make sure there is always one chunk of buffer between any lod, ie. lod2 does not border lod4 directly
				// this is usually the case anyway, but avoid edge cases
				// > needed for lod seam fix to work
				if (prev_bound0.x != prev_bound1.x) { // if prev_bound is actually valid
					bound0 = min(bound0, (prev_bound0 & parent_mask) - parent_sz);
					bound1 = max(bound1, (prev_bound1 & parent_mask) + parent_sz);
				}

				// clamp to heightmap bounds
				bound0 = clamp(bound0, int2(0), int2(MAP_SZ));
				bound1 = clamp(bound1, int2(0), int2(MAP_SZ));

				shad->set_uniform("lod_bound0", (float2)bound0);
				shad->set_uniform("lod_bound1", (float2)bound1);

				// interate over all chunks in bounds
				for (int y=bound0.y; y<bound1.y; y+=sz)
				for (int x=bound0.x; x<bound1.x; x+=sz) {
					// exclude chunks that are already covered by smaller LOD chunks
					if (  x >= prev_bound0.x && x < prev_bound1.x &&
						  y >= prev_bound0.y && y < prev_bound1.y )
						continue;
					
					if (dbg)
						r.dbgdraw.wire_quad(float3((float2)int2(x,y), 0), (float2)(float)sz, render::DebugDraw::COLS[wrap(lod, ARRLEN(render::DebugDraw::COLS))]);

					// draw chunk with this lod
					shad->set_uniform("offset", float2(int2(x,y)));
					shad->set_uniform("quad_size", quad_size);
					
					chunk();

					drawn_chunks++;
				}

				prev_bound0 = bound0;
				prev_bound1 = bound1;
			}
		}

		void gen_terrain_quad () {
			
			constexpr int sz = TERRAIN_CHUNK_SZ;
			
			std::vector<TerrainVertex> verts( (sz+1) * (sz+1) );
			std::vector<uint16_t>      indices( sz * sz * 6 );

			int vert_count=0;
			for (int y=0; y<sz+1; ++y)
			for (int x=0; x<sz+1; ++x) {
				verts[vert_count++] = TerrainVertex{{ (float)x, (float)y }};
			}

			int idx_count = 0;
			for (int y=0; y<sz; ++y)
			for (int x=0; x<sz; ++x) {
				uint16_t a = (uint16_t)( (y  )*(sz+1) + (x  ) );
				uint16_t b = (uint16_t)( (y  )*(sz+1) + (x+1) );
				uint16_t c = (uint16_t)( (y+1)*(sz+1) + (x+1) );
				uint16_t d = (uint16_t)( (y+1)*(sz+1) + (x  ) );

				if (((x ^ y) & 1) == 0) {
					// D---C
					// | / |
					// A---B
					indices[idx_count+0] = b;
					indices[idx_count+1] = c;
					indices[idx_count+2] = a;
					indices[idx_count+3] = a;
					indices[idx_count+4] = c;
					indices[idx_count+5] = d;
				}
				else {
					// D---C
					// | \ |
					// A---B
					indices[idx_count+0] = a;
					indices[idx_count+1] = b;
					indices[idx_count+2] = d;
					indices[idx_count+3] = b;
					indices[idx_count+4] = c;
					indices[idx_count+5] = d;
				}

				idx_count += 6;
			}

			terrain_chunk.upload(verts, indices);
			chunk_vertices = vert_count;
			chunk_indices = idx_count;
		}
		void render_terrain (View3D& view, OglRenderer& r) {
			ZoneScoped;

			drawn_chunks = 0;

			if (draw_terrain && shad_terrain->prog) {

				OGL_TRACE("render_terrain");
				
				PipelineState s;
				s.depth_test = true;
				s.blend_enable = false;
				r.state.set(s);

				glUseProgram(shad_terrain->prog);

				r.state.bind_textures(shad_terrain, {
					{"grid_tex", r.textures.grid, r.textures.sampler_normal},
				//	{"clouds", r.textures.clouds, r.textures.sampler_normal},
				//	{"heightmap", heightmap, r.sampler_heightmap},
					{"terrain_diffuse", r.textures.terrain_diffuse, r.textures.sampler_normal},
				});
			
				shad_terrain->set_uniform("inv_max_size", 1.0f / float2((float)MAP_SZ));

				glBindVertexArray(terrain_chunk.vao);
			
				lodded_chunks(view, r, shad_terrain, terrain_base_lod, dbg_lod, [this] () {
					glDrawElements(GL_TRIANGLES, chunk_indices, GL_UNSIGNED_SHORT, (void*)0);
				});
			}
		}
		//void render_ocean (Game& g, Renderer& r) {
		//	ZoneScoped;
		//
		//	if (draw_water && shad_water->prog) {
		//
		//		OGL_TRACE("render_water");
		//			
		//		PipelineState s;
		//		s.depth_test = true;
		//		s.blend_enable = true;
		//		r.state.set(s);
		//
		//		glUseProgram(shad_water->prog);
		//
		//		r.state.bind_textures(shad_water, {
		//			{ "opaque_fbo", {GL_TEXTURE_2D, r.passes.fbo_opaque_copy.col}, r.passes.fbo_sampler_bilin },
		//
		//			{ "clouds", r.clouds, r.sampler_normal },
		//		});
		//			
		//		r.ocean.set_uniforms(shad_water);
		//		shad_water->set_uniform("inv_max_size", 1.0f / float2((float)MAP_SZ));
		//
		//		glBindVertexArray(terrain_chunk.vao);
		//		
		//		lodded_chunks(g, shad_water, water_base_lod, dbg_lod, [this] () {
		//			glDrawElements(GL_TRIANGLES, chunk_indices, GL_UNSIGNED_SHORT, (void*)0);
		//		});
		//	}
		//}
	};
	TerrainRenderer terrain_renderer;

	struct SkyboxRenderer {
	
		Shader* shad = g_shaders.compile("skybox");

		struct Vertex {
			float3 pos;
		
			VERTEX_CONFIG(
				ATTRIB(FLT3, Vertex, pos),
			)
		};
		VertexBufferI skybox = vertex_bufferI<Vertex>("skybox");

		static constexpr float3 verts[] = {
			{-1,-1,-1},
			{+1,-1,-1},
			{-1,+1,-1},
			{+1,+1,-1},
			{-1,-1,+1},
			{+1,-1,+1},
			{-1,+1,+1},
			{+1,+1,+1},
		};
		static constexpr uint16_t indices[] = {
			// -Z 
			REND_QUAD_INDICES(0,1,3,2),
			// +X
			REND_QUAD_INDICES(3,1,5,7),
			// +Y
			REND_QUAD_INDICES(2,3,7,6),
			// -X
			REND_QUAD_INDICES(0,2,6,4),
			// -Y
			REND_QUAD_INDICES(1,0,4,5),
			// +Z
			REND_QUAD_INDICES(6,7,5,4),
		};

		SkyboxRenderer () {
			skybox.upload(verts, ARRLEN(verts), indices, ARRLEN(indices));
		}
	
		// draw after everything else
		// advantage: early-z avoids drawing skybox shader behind ground, might be better with scattering skybox shader
		// shader needs to use vec4(xyz, 0.0) clip coords to draw at infinity -> need to clip at far plane with GL_DEPTH_CLAMP
		void render_skybox_last (StateManager& state, OglRenderer& r) {
			if (!shad->prog) return;
		
			OGL_TRACE("draw_skybox");

			PipelineState s;
			s.depth_test   = true;
			s.depth_write  = false;
			s.blend_enable = false;
			state.set_no_override(s);

			glEnable(GL_DEPTH_CLAMP);

			glUseProgram(shad->prog);

			state.bind_textures(shad, {
			//	{ "clouds", r.textures.clouds, r.textures.sampler_normal }
			});
		
			glBindVertexArray(skybox.vao);

			glDrawElements(GL_TRIANGLES, ARRLEN(indices), GL_UNSIGNED_SHORT, (void*)0);

			glDisable(GL_DEPTH_CLAMP);
		}
	};
	SkyboxRenderer skybox;
	

	struct BuildingRenderer {
		Shader* shad = g_shaders.compile("buildings");

		VertexBufferI vbo = vertex_bufferI<LoadedMesh::Vertex>("buildings");

		BuildingRenderer (Assets& assets) {
			auto& mesh = assets.buildings[0]->mesh;
			vbo.upload(mesh.vertices, mesh.indices);
		}

		void draw (OglRenderer& r, App& app) {
			ZoneScoped;

			if (shad->prog) {

				OGL_TRACE("render_buildings");
				
				PipelineState s;
				s.depth_test = true;
				s.blend_enable = false;
				r.state.set(s);

				glUseProgram(shad->prog);

				r.state.bind_textures(shad, {
					{"tex", r.textures.house_diffuse, r.textures.sampler_normal},
				});

				glBindVertexArray(vbo.vao);

				for (auto& instance : app.entities.buildings) {
					shad->set_uniform("model2world", (float4x4)translate(instance->pos));

					auto& mesh = instance->asset->mesh;
					glDrawElements(GL_TRIANGLES, (GLsizei)mesh.indices.size(), GL_UNSIGNED_SHORT, (void*)0);
				}
			}
		}
	};
	BuildingRenderer building_renderer;

	struct Textures {
		//Texture2D clouds = load_texture<srgba8>("clouds", "textures/clouds.png");
		Texture2D grid = load_texture<srgba8>("grid", "textures/grid2.png");
		Texture2D terrain_diffuse = load_texture<srgb8>("terrain_diffuse", "textures/Rock_Moss_001_SD/Rock_Moss_001_basecolor.jpg");
	
		//Sampler sampler_heightmap = sampler("sampler_heightmap", FILTER_BILINEAR,  GL_REPEAT);
		Sampler sampler_normal = sampler("sampler_normal",    FILTER_MIPMAPPED, GL_REPEAT, true);

		Texture2D house_diffuse = load_texture<srgb8>("house_diffuse", "assets/house1/house.png");

		template <typename T>
		static auto load_texture (std::string_view gl_label, const char* filepath) {
			Texture2D tex = {gl_label};
			if (!upload_texture2D<T>(tex, filepath))
				assert(false);
			return tex;
		}
	};
	Textures textures;

	OglRenderer (Assets& assets): building_renderer{assets} {
		
	}
	
	virtual void begin (App& app) {
		
	}
	virtual void end (App& app) {
		ZoneScoped;
		
		{
			OGL_TRACE("draw ui");
		
			if (app.trigger_screenshot && !app.screenshot_hud) take_screenshot(app.input.window_size);
		
			// draw HUD

			app.draw_imgui();

			if (app.trigger_screenshot && app.screenshot_hud)  take_screenshot(app.input.window_size);
			app.trigger_screenshot = false;
		}

		{
			OGL_TRACE("setup");

			{
				//OGL_TRACE("set state defaults");

				state.wireframe          = gl_dbgdraw.wireframe;
				state.wireframe_no_cull  = gl_dbgdraw.wireframe_no_cull;
				state.wireframe_no_blend = gl_dbgdraw.wireframe_no_blend;

				state.set_default();

				glEnable(GL_LINE_SMOOTH);
				glLineWidth(gl_dbgdraw.line_width);
			}

			{
				common_ubo.set(app.view, lighting);
			
				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, gl_dbgdraw.indirect_vbo);

				gl_dbgdraw.update(app.input);
			}
		}
		
		lighting.sun_dir = float4(app.sun_dir, 0.0);

		passes.update(app.input.window_size);

		passes.begin_primary();
		{
			OGL_TRACE("draw opaque");

			terrain_renderer.render_terrain(app.view, *this);
		
			building_renderer.draw(*this, app);

			skybox.render_skybox_last(state, *this);
		}

		passes.copy_primary_for_distortion();
		{
			OGL_TRACE("draw transparent");

			//terrain_renderer.render_ocean(g, *this);

			gl_dbgdraw.render(state, dbgdraw);
		}

		// 
		passes.postprocess(state, app.input.window_size);
		{
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

std::unique_ptr<Renderer> create_ogl_backend (Assets& assets) {
	return std::make_unique<ogl::OglRenderer>(assets);
}
