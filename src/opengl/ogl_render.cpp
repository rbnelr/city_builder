#pragma once
#include "common.hpp"
#include "app.hpp"
#include "engine/opengl.hpp"
#include "agnostic_render.hpp"
#include "gl_dbgdraw.hpp"
#include "render_passes.hpp"

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

	void push_path (float3 a, float3 b, float width, float4 col) {
		uint16_t idx = (uint16_t)verticies.size();

		float2 dir2d = normalizesafe((float2)b - (float2)a);
		float2 norm = rotate90(dir2d) * width*0.5f;

		float3 a0 = float3((float2)a + norm, a.z + 0.1f);
		float3 a1 = float3((float2)a - norm, a.z + 0.1f);
		float3 b0 = float3((float2)b + norm, b.z + 0.1f);
		float3 b1 = float3((float2)b - norm, b.z + 0.1f);

		auto* pv = push_back(verticies, 4);
		pv[0] = { a0, float2(0,0), col };
		pv[1] = { a1, float2(1,0), col };
		pv[2] = { b1, float2(1,1), col };
		pv[3] = { b0, float2(0,1), col };

		render::shapes::push_quad_indices<uint16_t>(indices, idx+0u, idx+1u, idx+2u, idx+3u);
	}

	void update (Network& net) {

		verticies.clear();
		verticies.shrink_to_fit();
		indices.clear();
		indices.shrink_to_fit();

		for (auto& seg : net.segments) {
			push_path(seg->a->pos, seg->b->pos, 12, lrgba(lrgb(0.05f), 1.0f));
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

			gl_dbgdraw.imgui();

			lod.imgui();

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

	struct BuildingRenderer {
		size_t drawn_vertices;
		size_t drawn_indices;

	#if 0
	//// Normal drawcalls
	/*
	Normal Drawcalls:
	Buildings NxN:  20,    500,    1000
	cpu ms:         0.1,    40,    170
	gpu fps:        ~144,   20,    6
	*/
		Shader* shad = g_shaders.compile("buildings", {{"MODE", "0"}});
		
		struct MeshBuf {
			struct Lod {
				VertexBufferI buf;
				size_t vertex_count;
				size_t index_count;
			};
			std::vector<Lod> lods;

			MeshBuf (AssetMesh& mesh) {
				for (auto& lod : mesh.mesh_lods) {
					auto& buf = lods.emplace_back(Lod{ vertex_bufferI<Mesh::Vertex>("building"), lod.vertices.size(), lod.indices.size() });
					buf.buf.upload(lod.vertices, lod.indices);
				}
			}
		};
		std::unordered_map<BuildingAsset*, MeshBuf> meshes;

		void reload (Assets& assets) {
			meshes.clear();
			for (auto& asset : assets.buildings) {
				meshes.emplace(asset.get(), asset->mesh);
			}
		}

		void draw (OglRenderer& r, App& app) {
			ZoneScoped;

			drawn_vertices = 0;
			drawn_indices = 0;

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

				int i=0;
				for (auto& instance : app.entities.buildings) {
					shad->set_uniform("model2world", (float4x4)translate(instance->pos));

					auto it = meshes.find(instance->asset);
					assert(it != meshes.end());
					auto& buf = it->second;
					
					int lod = r.lod.pick_lod(app.view, instance->pos, 32);
					lod = min(lod, (int)buf.lods.size()-1); // don't allow obj to be not drawn due to lod

					if (lod >= buf.lods.size())
						continue;

					glBindVertexArray(buf.lods[lod].buf.vao);
					
					glDrawElements(GL_TRIANGLES, (GLsizei)buf.lods[lod].index_count, GL_UNSIGNED_SHORT, (void*)0);

					drawn_vertices += buf.lods[lod].vertex_count;
					drawn_indices += buf.lods[lod].index_count;
				}
			}

			ImGui::Text("drawn vertices: %.3fM (indices: %.3fM)", drawn_vertices / 1000000.0f, drawn_indices / 1000000.0f);
		}
	#elif 0
	//// Instanced drawing (still streaming instance data)
	/*
	Instanced with instance streaming: 2x fps   10x cpu speedup
	Buildings NxN:  20,    500,    1000
	cpu ms:         0.038,  3,     18
	gpu fps:        ~144,   40,    12
	*/
		Shader* shad = g_shaders.compile("buildings", {{"MODE", "1"}});
		
		struct BuildingInstance {
			float3 pos;
			float  rot;
			
			VERTEX_CONFIG_INSTANCED(
				ATTRIB(FLT3, BuildingInstance, pos),
				ATTRIB(FLT , BuildingInstance, rot),
			)
		};

		typedef Mesh::Vertex vert_t;
		typedef uint16_t     idx_t;

		struct MeshBuf {
			VertexBufferInstancedI buf = vertex_buffer_instacedI<Mesh::Vertex, BuildingInstance>("building");

			struct Lod {
				size_t vertex_base;
				size_t vertex_count;
				size_t index_base;
				size_t index_count;
			};
			std::vector<Lod> lods;

			MeshBuf (AssetMesh& mesh) {
				size_t i=0, v=0;
				for (auto& lod : mesh.mesh_lods) {
					auto& l = lods.emplace_back();

					l.vertex_base = v;
					l.vertex_count = lod.vertices.size();
					l.index_base = i;
					l.index_count = lod.indices.size();

					v += l.vertex_count;
					i += l.index_count;
				}
				
				glBindBuffer(GL_ARRAY_BUFFER, buf.vbo);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buf.ebo);

				glBufferData(GL_ARRAY_BUFFER,         v * sizeof(vert_t), nullptr, GL_STATIC_DRAW);
				glBufferData(GL_ELEMENT_ARRAY_BUFFER, i * sizeof(idx_t),  nullptr, GL_STATIC_DRAW);
				
				for (size_t i=0; i<lods.size(); ++i) {
					auto& asset_lod = mesh.mesh_lods[i];
					auto& lod       = lods[i];
					glBufferSubData(GL_ARRAY_BUFFER,         lod.vertex_base * sizeof(vert_t), lod.vertex_count * sizeof(vert_t), asset_lod.vertices.data());
					glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, lod.index_base * sizeof(idx_t),   lod.index_count * sizeof(idx_t),   asset_lod.indices.data());
				}

				glBindBuffer(GL_ARRAY_BUFFER, 0);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
			}
		};
		std::unordered_map<BuildingAsset*, MeshBuf> asset_meshes;

		void reload (Assets& assets) {
			for (auto& asset : assets.buildings) {
				asset_meshes.emplace(asset.get(), asset->mesh);
			}
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

				struct Instances {
					std::vector< std::vector<BuildingInstance> > lod_instances;
				};
				std::unordered_map<BuildingAsset*, Instances> asset_instances;

				asset_instances.reserve(asset_meshes.size());
				for (auto& asset : app.assets.buildings) {
					auto& ins = asset_instances.emplace(asset.get(), Instances{}).first->second;
					ins.lod_instances.resize(asset->mesh.mesh_lods.size());
				}

				for (auto& entity : app.entities.buildings) {
					auto& instances = asset_instances.find(entity->asset)->second;
					int lods = (int)instances.lod_instances.size();

					int lod = r.lod.pick_lod(app.view, entity->pos, 32);
					lod = min(lod, lods-1); // don't allow obj to be not drawn due to lod

					if (lod < lods) {
						auto& instance = instances.lod_instances[lod].emplace_back();
						instance.pos = entity->pos;
						instance.rot = 0;
					}
				}

				drawn_vertices = 0;
				drawn_indices = 0;
				
				for (auto& kv : asset_instances) {
					auto* asset = kv.first;
					auto& mesh_buf = asset_meshes.find(asset)->second;

					for (int lod=0; lod<(int)kv.second.lod_instances.size(); ++lod) {
						auto& mesh_lod = mesh_buf.lods[lod];
						auto& instances = kv.second.lod_instances[lod];

						mesh_buf.buf.stream_instances(instances);

						glBindVertexArray(mesh_buf.buf.vao);
						glDrawElementsInstancedBaseVertex(GL_TRIANGLES, (GLsizei)mesh_lod.index_count, GL_UNSIGNED_SHORT, (void*)(mesh_lod.index_base * sizeof(idx_t)), (GLsizei)instances.size(), (GLint)(mesh_lod.vertex_base));

						drawn_vertices += mesh_lod.vertex_count * instances.size();
						drawn_indices  += mesh_lod.index_count  * instances.size();
					}
				}

				ImGui::Text("drawn vertices: %.3fM (indices: %.3fM)", drawn_vertices / 1000000.0f, drawn_indices / 1000000.0f);
			}
		}
	#elif 0
	//// glMultiDrawElementsIndirect drawing (cpu sided indirect buffer generation)
	/*
	Instanced with instance streaming: 2x fps   10x cpu speedup
	Buildings NxN:  20,    500,    1000
	cpu ms:         0.038,  3,     18
	gpu fps:        ~144,   40,    12
	*/
		Shader* shad = g_shaders.compile("buildings", {{"MODE", "1"}});
		
		struct BuildingInstance {
			float3 pos;
			float  rot;
			
			VERTEX_CONFIG_INSTANCED(
				ATTRIB(FLT3, BuildingInstance, pos),
				ATTRIB(FLT , BuildingInstance, rot),
			)
		};

		typedef Mesh::Vertex vert_t;
		typedef uint16_t     idx_t;

		// All meshes/lods vbo (indexed) GL_ARRAY_BUFFER / GL_ELEMENT_ARRAY_BUFFER
		// All entities instance data GL_ARRAY_BUFFER
		VertexBufferInstancedI vbo = vertex_buffer_instacedI<Mesh::Vertex, BuildingInstance>("building");

		// All entities lodded draw commands
		Vbo indirect_vbo = {"DebugDraw.indirect_draw"};

		struct Lod {
			uint32_t vertex_base;
			uint32_t vertex_count;
			uint32_t index_base;
			uint32_t index_count;
		};
		std::unordered_map<BuildingAsset*, std::vector<Lod>> mesh_lods;

		void reload (Assets& assets) {
			glBindBuffer(GL_ARRAY_BUFFER, vbo.vbo);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo.ebo);

			mesh_lods.clear();
			
			uint32_t indices=0, vertices=0;
			for (auto& asset : assets.buildings) {
				auto& lods = mesh_lods.emplace(asset.get(), std::vector<Lod>{}).first->second;

				for (auto& lod : asset->mesh.mesh_lods) {
					auto& l = lods.emplace_back();

					l.vertex_base  = (uint32_t)vertices;
					l.vertex_count = (uint32_t)lod.vertices.size();
					l.index_base  = (uint32_t)indices;
					l.index_count = (uint32_t)lod.indices.size();
					
					vertices += l.vertex_count;
					indices += l.index_count;
				}
			}
			
			glBufferData(GL_ARRAY_BUFFER,         vertices * sizeof(vert_t), nullptr, GL_STATIC_DRAW);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices  * sizeof(idx_t),  nullptr, GL_STATIC_DRAW);
			
			for (auto& asset : assets.buildings) {
				auto& lods = mesh_lods[asset.get()];

				for (size_t i=0; i<lods.size(); ++i) {
					auto& asset_lod = asset->mesh.mesh_lods[i];
					auto& lod       = lods[i];
					glBufferSubData(GL_ARRAY_BUFFER,         lod.vertex_base * sizeof(vert_t), lod.vertex_count * sizeof(vert_t), asset_lod.vertices.data());
					glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, lod.index_base * sizeof(idx_t),   lod.index_count * sizeof(idx_t),   asset_lod.indices.data());
				}
			}

			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
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

				std::vector<BuildingInstance> instances;
				std::vector<glDrawElementsIndirectCommand> cmds;

				instances.resize(app.entities.buildings.size());
				cmds.resize(app.entities.buildings.size());

				drawn_vertices = 0;
				drawn_indices = 0;

				for (uint32_t i=0; i<(uint32_t)app.entities.buildings.size(); ++i) {
					auto& entity = app.entities.buildings[i];

					auto& lods = mesh_lods.find(entity->asset)->second;
					int lod_count = (int)lods.size();

					int lod = r.lod.pick_lod(app.view, entity->pos, 32);
					lod = min(lod, lod_count-1); // don't allow obj to be not drawn due to lod

					if (lod < lod_count) {
						instances[i].pos = entity->pos;
						instances[i].rot = 0;

						cmds[i].count         = lods[lod].index_count;
						cmds[i].instanceCount = 1;
						cmds[i].firstIndex    = lods[lod].index_base;
						cmds[i].baseVertex    = lods[lod].vertex_base;
						cmds[i].baseInstance  = i;

						drawn_vertices += lods[lod].vertex_count;
						drawn_indices  += lods[lod].index_count;
					}
				}

				vbo.stream_instances(instances);
				stream_buffer(GL_ARRAY_BUFFER, indirect_vbo, (GLsizeiptr)(cmds.size() * sizeof(glDrawElementsIndirectCommand)), cmds.data(), GL_STREAM_DRAW);
				
				glBindVertexArray(vbo.vao);
				glBindBuffer(GL_DRAW_INDIRECT_BUFFER, indirect_vbo);

				glMultiDrawElementsIndirect(GL_TRIANGLES, GL_UNSIGNED_SHORT, (void*)0, (GLsizei)cmds.size(), 0);
				
				glBindVertexArray(0);
				glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);


				ImGui::Text("drawn vertices: %.3fM (indices: %.3fM)", drawn_vertices / 1000000.0f, drawn_indices / 1000000.0f);
			}
		}
	#else
	//// glMultiDrawElementsIndirect drawing (cpu sided indirect buffer generation)
	/*
	Instanced with instance streaming: 2x fps   10x cpu speedup
	Buildings NxN:  20,    500,    1000
	cpu ms:         0.038,  3,     18
	gpu fps:        ~144,   40,    12
	*/
		static constexpr uint32_t COMPUTE_GROUPSZ = 512;
		
		Shader* shad = g_shaders.compile("buildings", {{"MODE", "2"}});
		Shader* shad_lod_cull = g_shaders.compile("lod_cull", { {"GROUPSZ", prints("%d", COMPUTE_GROUPSZ)} }, { { shader::COMPUTE_SHADER } });

		typedef Mesh::Vertex vert_t;
		typedef uint16_t     idx_t;
		
		struct MeshInstance {
			int    mesh_id;
			float3 pos;
			float  rot;
			
			VERTEX_CONFIG_INSTANCED(
				ATTRIB(INT , MeshInstance, mesh_id),
				ATTRIB(FLT3, MeshInstance, pos),
				ATTRIB(FLT , MeshInstance, rot),
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
		VertexBufferInstancedI vbo = vertex_buffer_instacedI<Mesh::Vertex, MeshInstance>("meshes");
		
		Ssbo ssbo_mesh_info = {"mesh_info"};
		Ssbo ssbo_mesh_lod_info = {"mesh_lod_info"};

		// All entities lodded draw commands
		Vbo indirect_vbo = {"DebugDraw.indirect_draw"};

		std::unordered_map<BuildingAsset*, int> asset2mesh_id;

		BuildingRenderer () {
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

		void reload (Assets& assets) {
			std::vector<MeshInfo> mesh_infos;
			std::vector<MeshLodInfo> mesh_lod_infos;

			glBindBuffer(GL_ARRAY_BUFFER, vbo.vbo);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo.ebo);
			
			uint32_t indices=0, vertices=0;
			uint32_t mesh_id=0;
			uint32_t mesh_lod_id=0;
			for (auto& asset : assets.buildings) {
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
			for (auto& asset : assets.buildings) {
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

		void draw (OglRenderer& r, App& app) {
			ZoneScoped;
			OGL_TRACE("render_buildings");

			if (app.entities.changes) {
				std::vector<MeshInstance> instances;
				instances.resize(app.entities.buildings.size());

				drawn_vertices = 0;
				drawn_indices = 0;

				for (uint32_t i=0; i<(uint32_t)app.entities.buildings.size(); ++i) {
					auto& entity = app.entities.buildings[i];

					//instances[i].mesh_id = asset2mesh_id[entity->asset];
					//instances[i].pos = entity->pos;
					//instances[i].rot = 0;
					instances[i].mesh_id = asset2mesh_id[entity.asset];
					instances[i].pos = entity.pos;
					instances[i].rot = 0;
				}

				vbo.stream_instances(instances);
			}
		
			uint32_t entities = (uint32_t)app.entities.buildings.size();
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
					{"tex", r.textures.house_diffuse, r.textures.sampler_normal},
				});

				glMultiDrawElementsIndirect(GL_TRIANGLES, GL_UNSIGNED_SHORT, (void*)0, entities, 0);
				
				glBindVertexArray(0);
				glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
			}

			ImGui::Text("drawn vertices: %.3fM (indices: %.3fM)", drawn_vertices / 1000000.0f, drawn_indices / 1000000.0f); // TODO: ????? How to get this info?
			
		}
	#endif
	};
	BuildingRenderer building_renderer;

	struct Textures {
		//Texture2D clouds = load_texture<srgba8>("clouds", "textures/clouds.png");
		Texture2D grid = load_texture<srgba8>("grid", "misc/grid2.png");
		Texture2D terrain_diffuse = load_texture<srgb8>("terrain_diffuse", "misc/Rock_Moss_001_SD/Rock_Moss_001_basecolor.jpg");
	
		//Sampler sampler_heightmap = sampler("sampler_heightmap", FILTER_BILINEAR,  GL_REPEAT);
		Sampler sampler_normal = sampler("sampler_normal",    FILTER_MIPMAPPED, GL_REPEAT, true);

		Texture2D house_diffuse = load_texture<srgb8>("house_diffuse", "buildings/house.png");

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
		
	}
	virtual void end (App& app) {
		ZoneScoped;

		if (app.assets.assets_reloaded) {
			building_renderer.reload(app.assets);
		}

		network_renderer.update(app.net);

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

			network_renderer.render(state);

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

std::unique_ptr<Renderer> create_ogl_backend () {
	return std::make_unique<ogl::OglRenderer>();
}
