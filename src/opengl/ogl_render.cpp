#pragma once
#include "common.hpp"
#include "app.hpp"
#include "engine/opengl.hpp"
#include "agnostic_render.hpp"
#include "gl_dbgdraw.hpp"
#include "render_passes.hpp"
#include "engine/text_render.hpp"

namespace ogl {

struct TriRenderer {
	Shader* shad  = g_shaders.compile("tris");

	//Texture2D tex = texture2D<srgba8>("logo", "textures/Opengl-logo.png");

	Sampler sampler_normal = sampler("sampler_normal", FILTER_MIPMAPPED, GL_REPEAT, false);

	struct Vertex {
		float3 pos;
		float2 uv;
		float4 col;

		VERTEX_CONFIG(
			ATTRIB(FLT3, Vertex, pos),
			ATTRIB(FLT2, Vertex, uv),
			ATTRIB(FLT4, Vertex, col),
		)
	};

	VertexBufferI vbo_tris = vertex_bufferI<Vertex>("TriRenderer.Vertex");

	std::vector<Vertex>   verticies;
	std::vector<uint16_t> indices;

	void push_path (float2 forw, float2 right, float3 a, float3 b, float width, float2 offsets, float shift, float z, float4 col) {
		uint16_t idx = (uint16_t)verticies.size();
		
		float2 half_width = width*0.5f;

		float3 a0 = a + float3(right * (shift - half_width) + forw * offsets[0], z);
		float3 a1 = a + float3(right * (shift + half_width) + forw * offsets[0], z);
		float3 b0 = b + float3(right * (shift - half_width) - forw * offsets[1], z);
		float3 b1 = b + float3(right * (shift + half_width) - forw * offsets[1], z);

		auto* pv = push_back(verticies, 4);
		pv[0] = { a0, float2(0,0), col };
		pv[1] = { a1, float2(1,0), col };
		pv[2] = { b1, float2(1,1), col };
		pv[3] = { b0, float2(0,1), col };

		render::shapes::push_quad_indices<uint16_t>(indices, idx+0u, idx+1u, idx+2u, idx+3u);
	}
	void push_node (float3 center, float radius, float z, float4 col) {
		uint16_t idx = (uint16_t)verticies.size();
		
		auto* pv = push_back(verticies, 4);
		pv[0] = { center + float3(-radius, -radius, z), float2(0,0), col };
		pv[1] = { center + float3(+radius, -radius, z), float2(1,0), col };
		pv[2] = { center + float3(+radius, +radius, z), float2(1,1), col };
		pv[3] = { center + float3(-radius, +radius, z), float2(0,1), col };

		render::shapes::push_quad_indices<uint16_t>(indices, idx+0u, idx+1u, idx+2u, idx+3u);
	}

	void update (network::Network& net) {

		verticies.clear();
		verticies.shrink_to_fit();
		indices.clear();
		indices.shrink_to_fit();

		for (auto& seg : net.segments) {
			auto v = seg->clac_seg_vecs();
			float width = seg->layout->width;
			float2 offsets = { seg->node_a->radius, seg->node_b->radius };

			push_path(v.forw, v.right, seg->node_a->pos, seg->node_b->pos, width, offsets, 0.0f, 0.05f, lrgba(lrgb(0.05f), 1.0f));
			
			for (auto& lane : seg->layout->lanes) {
				push_path(v.forw, v.right, seg->node_a->pos, seg->node_b->pos, lane.width, offsets, lane.shift, 0.10f, lrgba(lrgb(0.08f), 1.0f));
			}
		}

		for (auto& node : net.nodes) {
			push_node(node->pos, node->radius, 0.05f, lrgba(lrgb(0.05f), 1.0f));
		}
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
					//{ "tex", tex, sampler_normal }
				});

				PipelineState s;
				s.blend_enable = true;
				state.set(s);

				glBindVertexArray(vbo_tris.vao);
				glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_UNSIGNED_SHORT, (void*)0);
			}
		}

		glBindVertexArray(0);
	}
};

struct OglRenderer : public Renderer {
	SERIALIZE_NONE(OglRenderer)
	
	virtual void imgui (App& app) {
		if (imgui_Header("Renderer", true)) {
			
			passes.imgui();

		#if OGL_USE_REVERSE_DEPTH
			ImGui::Checkbox("reverse_depth", &ogl::reverse_depth);
		#endif

			gl_dbgdraw.imgui(g_dbgdraw.text);

			lod.imgui();

			lighting.imgui();
			terrain_renderer.imgui();

			ImGui::PopID();
		}
	}

	StateManager state;

	glDebugDraw gl_dbgdraw;
	
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
				g_dbgdraw.wire_quad(float3(0), (float2)MAP_SZ, lrgba(0,0,0,1));

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
						g_dbgdraw.wire_quad(float3((float2)int2(x,y), 0), (float2)(float)sz, g_dbgdraw.COLS[wrap(lod, ARRLEN(g_dbgdraw.COLS))]);

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

	TriRenderer network_renderer;

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
	
	LOD_Func lod;

	template <typename ASSET_T>
	struct EntityRenderer {
		typedef typename decltype(ASSET_T().mesh)::vert_t vert_t;
		typedef typename decltype(ASSET_T().mesh)::idx_t  idx_t;

		static constexpr uint32_t COMPUTE_GROUPSZ = 512;

		Shader* shad = g_shaders.compile("buildings");
		Shader* shad_lod_cull = g_shaders.compile("lod_cull", { {"GROUPSZ", prints("%d", COMPUTE_GROUPSZ)} }, { { shader::COMPUTE_SHADER } });

		struct MeshInstance {
			int    mesh_id;
			float3 pos;
			float  rot;
			float3 col; // Just for debug?
			
			VERTEX_CONFIG_INSTANCED(
				ATTRIB(INT , MeshInstance, mesh_id),
				ATTRIB(FLT3, MeshInstance, pos),
				ATTRIB(FLT , MeshInstance, rot),
				ATTRIB(FLT3, MeshInstance, col),
			)
		};
		struct MeshInfo {
			uint32_t mesh_lod_id; // index of MeshLodInfos [lods]
			uint32_t lods;
		};
		struct MeshLodInfo {
			uint32_t vertex_base;
			uint32_t vertex_count;
			uint32_t index_base;
			uint32_t index_count;
		};

		// All meshes/lods vbo (indexed) GL_ARRAY_BUFFER / GL_ELEMENT_ARRAY_BUFFER
		// All entities instance data GL_ARRAY_BUFFER
		VertexBufferInstancedI vbo = vertex_buffer_instancedI<vert_t, MeshInstance>("meshes");
		
		Ssbo ssbo_mesh_info = {"mesh_info"};
		Ssbo ssbo_mesh_lod_info = {"mesh_lod_info"};

		// All entities lodded draw commands
		Vbo indirect_vbo = {"DebugDraw.indirect_draw"};

		std::unordered_map<ASSET_T*, int> asset2mesh_id;
		
		uint32_t entities;
		
		EntityRenderer () {
			//int3 sz = -1;
			//glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 0, &sz.x);
			//glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 1, &sz.y);
			//glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 2, &sz.z);
			//
			//int3 cnt = -1;
			//glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 0, &cnt.x);
			//glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 1, &cnt.y);
			//glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 2, &cnt.z);
		}

		void upload_meshes (Assets::Collection<ASSET_T> const& assets) {
			std::vector<MeshInfo> mesh_infos;
			std::vector<MeshLodInfo> mesh_lod_infos;

			glBindBuffer(GL_ARRAY_BUFFER, vbo.vbo);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo.ebo);
			
			uint32_t indices=0, vertices=0;
			uint32_t mesh_id=0;
			uint32_t mesh_lod_id=0;
			for (auto& asset : assets) {
				asset2mesh_id.emplace(asset.get(), mesh_id++);

				auto& info = mesh_infos.emplace_back();
				info.mesh_lod_id = mesh_lod_id;
				info.lods = (uint32_t)asset->mesh.mesh_lods.size();

				for (auto& lod : asset->mesh.mesh_lods) {
					auto& l = mesh_lod_infos.emplace_back();

					l.vertex_base  = (uint32_t)vertices;
					l.vertex_count = (uint32_t)lod.vertices.size();
					l.index_base  = (uint32_t)indices;
					l.index_count = (uint32_t)lod.indices.size();
					
					vertices += l.vertex_count;
					indices += l.index_count;

					mesh_lod_id++;
				}
			}
			
			glBufferData(GL_ARRAY_BUFFER,         vertices * sizeof(vert_t), nullptr, GL_STATIC_DRAW);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices  * sizeof(idx_t),  nullptr, GL_STATIC_DRAW);
			
			mesh_id=0;
			for (auto& asset : assets) {
				for (uint32_t i=0; i<asset->mesh.mesh_lods.size(); ++i) {
					auto& asset_lod = asset->mesh.mesh_lods[i];
					auto& lod = mesh_lod_infos[mesh_id++];

					glBufferSubData(GL_ARRAY_BUFFER,         lod.vertex_base * sizeof(vert_t), lod.vertex_count * sizeof(vert_t), asset_lod.vertices.data());
					glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, lod.index_base * sizeof(idx_t),   lod.index_count * sizeof(idx_t),   asset_lod.indices.data());
				}
			}

			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

			upload_buffer(GL_SHADER_STORAGE_BUFFER, ssbo_mesh_info, sizeof(mesh_infos[0]) * mesh_infos.size(), mesh_infos.data(), GL_STATIC_DRAW);
			upload_buffer(GL_SHADER_STORAGE_BUFFER, ssbo_mesh_lod_info, sizeof(mesh_lod_infos[0]) * mesh_lod_infos.size(), mesh_lod_infos.data(), GL_STATIC_DRAW);
		}

		template <typename FUNC>
		void update_instances (FUNC push_instances) {
			ZoneScoped;
			OGL_TRACE("update instances");

			auto instances = push_instances();
			vbo.stream_instances(instances);

			entities = (uint32_t)instances.size();
		}

		void draw (OglRenderer& r, Texture2D& tex) {
			ZoneScoped;
			OGL_TRACE("draw entities");

			{
				OGL_TRACE("lod_cull compute");
				
				glBindBuffer(GL_ARRAY_BUFFER, indirect_vbo);
				glBufferData(GL_ARRAY_BUFFER, entities * sizeof(glDrawElementsIndirectCommand), nullptr, GL_STREAM_DRAW);
				glBindBuffer(GL_ARRAY_BUFFER, 0);
			
				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, indirect_vbo);
				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, vbo.instances);
				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, ssbo_mesh_info);
				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, ssbo_mesh_lod_info);
				
				glUseProgram(shad_lod_cull->prog);

				shad_lod_cull->set_uniform("entities", entities);

				uint32_t groups_x = (entities + COMPUTE_GROUPSZ-1) / COMPUTE_GROUPSZ;
				glDispatchCompute(groups_x, 1, 1);
				glMemoryBarrier(GL_COMMAND_BARRIER_BIT);
			
				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, 0);
				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, 0);
				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, 0);
				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, 0);
			}

			if (shad->prog) {
				OGL_TRACE("draw indirect");

				PipelineState s;
				s.depth_test = true;
				s.blend_enable = false;
				r.state.set(s);

				glUseProgram(shad->prog);

				glBindVertexArray(vbo.vao);
				glBindBuffer(GL_DRAW_INDIRECT_BUFFER, indirect_vbo);

				r.state.bind_textures(shad, {
					{"tex", tex, r.textures.sampler_normal},
				});

				glMultiDrawElementsIndirect(GL_TRIANGLES, GL_UNSIGNED_SHORT, (void*)0, entities, 0);
				
				glBindVertexArray(0);
				glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
			}

			//ImGui::Text("drawn vertices: %.3fM (indices: %.3fM)", drawn_vertices / 1000000.0f, drawn_indices / 1000000.0f); // TODO: ????? How to get this info?
		}
	};

	EntityRenderer<BuildingAsset> building_renderer;
	EntityRenderer<CarAsset> car_renderer;

	struct Textures {
		//Texture2D clouds = load_texture<srgba8>("clouds", "textures/clouds.png");
		Texture2D grid = load_texture<srgba8>("grid", "misc/grid2.png");
		Texture2D terrain_diffuse = load_texture<srgb8>("terrain_diffuse", "misc/Rock_Moss_001_SD/Rock_Moss_001_basecolor.jpg");
	
		//Sampler sampler_heightmap = sampler("sampler_heightmap", FILTER_BILINEAR,  GL_REPEAT);
		Sampler sampler_normal = sampler("sampler_normal", FILTER_MIPMAPPED, GL_REPEAT, true);

		Texture2D house_diffuse = load_texture<srgb8>("house_diffuse", "buildings/house.png");
		Texture2D car_diffuse = load_texture<srgb8>("car_diffuse", "cars/car.png");

		template <typename T>
		static auto load_texture (std::string_view gl_label, const char* filepath) {
			Texture2D tex = {gl_label};
			if (!upload_texture2D<T>(tex, prints("assets/%s", filepath).c_str()))
				assert(false);
			return tex;
		}
	};
	Textures textures;

	OglRenderer () {
		
	}
	
	virtual void begin (App& app) {
		ZoneScoped;
		gl_dbgdraw.gl_text_render.begin(g_dbgdraw.text); // upload text and init data structures to allow text printing
	}
	virtual void end (App& app) {
		ZoneScoped;

		if (app.assets.assets_reloaded) {
			building_renderer.upload_meshes(app.assets.buildings);
			car_renderer.upload_meshes(app.assets.cars);
		}

		if (app.entities.buildings_changed) {
			network_renderer.update(app.net);

			building_renderer.update_instances([&] () {
				std::vector<decltype(building_renderer)::MeshInstance> instances;
				instances.resize(app.entities.buildings.size());

				for (uint32_t i=0; i<(uint32_t)app.entities.buildings.size(); ++i) {
					auto& entity = app.entities.buildings[i];

					instances[i].mesh_id = building_renderer.asset2mesh_id[entity->asset];
					instances[i].pos = entity->pos;
					instances[i].rot = entity->rot;
					instances[i].col = 1;
				}

				return instances;
			});
		}
		{
			car_renderer.update_instances([&] () {
				std::vector<decltype(car_renderer)::MeshInstance> instances;
				instances.resize(app.entities.citizens.size());

				for (uint32_t i=0; i<(uint32_t)app.entities.citizens.size(); ++i) {
					auto& entity = app.entities.citizens[i];

					// TODO: network code shoud ensure length(dir) == CAR_SIZE
					float3 dir = entity->front_pos - entity->back_pos;
					float3 center = entity->front_pos - normalizesafe(dir) * CAR_SIZE/2;
					float ang = angle2d((float2)dir);

					instances[i].mesh_id = car_renderer.asset2mesh_id[entity->asset];
					instances[i].pos = center;
					instances[i].rot = ang;
					instances[i].col = entity->col;
				}

				return instances;
			});
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
		
			building_renderer.draw(*this, textures.house_diffuse);
			car_renderer.draw(*this, textures.car_diffuse);

			network_renderer.render(state);

			skybox.render_skybox_last(state, *this);
		}

		passes.copy_primary_for_distortion();
		{
			OGL_TRACE("draw transparent");

			//terrain_renderer.render_ocean(g, *this);

			gl_dbgdraw.render(state, g_dbgdraw);
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

std::unique_ptr<Renderer> create_ogl_backend () {
	return std::make_unique<ogl::OglRenderer>();
}
