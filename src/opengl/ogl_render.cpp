#pragma once
#include "common.hpp"
#include "app.hpp"
#include "engine/opengl.hpp"
#include "agnostic_render.hpp"
#include "gl_dbgdraw.hpp"
#include "textures.hpp"
#include "render_passes.hpp"
#include "engine/text_render.hpp"

namespace ogl {
	
template <typename VERT_T, typename IDX_T=uint16_t>
struct Mesh {
	std::vector<VERT_T> verticies;
	std::vector<IDX_T>  indices;

	void clear () {
		verticies.clear();
		verticies.shrink_to_fit();
		indices.clear();
		indices.shrink_to_fit();
	}

	// CCW vertices
	void _FORCEINLINE push_quad (VERT_T const& a, VERT_T const& b, VERT_T const& c, VERT_T const& d) {
		//assert(verticies.size() < (IDX_T)-1)
		IDX_T idx = (IDX_T)verticies.size();
		
		auto* pv = push_back(verticies, 4);
		pv[0] = a;
		pv[1] = b;
		pv[2] = c;
		pv[3] = d;

		render::shapes::push_quad_indices<IDX_T>(indices, idx+0u, idx+1u, idx+2u, idx+3u);
	}


	void _FORCEINLINE push_tri (VERT_T const& a, VERT_T const& b, VERT_T const& c) {
		//assert(verticies.size() < (IDX_T)-1)
		IDX_T idx = (IDX_T)verticies.size();
		
		auto* pv = push_back(verticies, 3);
		pv[0] = a;
		pv[1] = b;
		pv[2] = c;
		
		auto* pi = push_back(indices, 3);
		pi[0] = idx+0u;
		pi[1] = idx+1u;
		pi[2] = idx+2u;
	}

	static constexpr GLenum GL_IDX_T = get_gl_idx_type<IDX_T>();
	
	void stream_draw (VertexBufferI& vbo) {
		vbo.stream(verticies, indices);
		
		if (indices.size() > 0) {
			glBindVertexArray(vbo.vao);
			glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_IDX_T, (void*)0);
		}
	}
};

struct TriRenderer {
	Shader* shad  = g_shaders.compile("tris");

	struct Vertex {
		float3 pos;
		float2 uv;
		float4 col;

		VERTEX_CONFIG(
			ATTRIB(FLT,3, Vertex, pos),
			ATTRIB(FLT,2, Vertex, uv),
			ATTRIB(FLT,4, Vertex, col),
		)
	};

	VertexBufferI vbo = vertex_bufferI<Vertex>("TriRenderer.Vertex");

	Mesh<Vertex, uint32_t> mesh;

	void render (StateManager& state, Textures& texs) {
		OGL_TRACE("TriRenderer");
		ZoneScoped;

		if (shad->prog) {
			glUseProgram(shad->prog);

			state.bind_textures(shad, {
				//{ "turn_arrows", tex.turn_arrows, tex.sampler_normal }
			});

			PipelineState s;
			s.blend_enable = true;
			state.set(s);

			mesh.stream_draw(vbo);
		}

		glBindVertexArray(0);
	}
};

struct TerrainRenderer {
	static constexpr int TERRAIN_CHUNK_SZ = 32; // 128 is max with GL_UNSIGNED_SHORT indices

	Shader* shad_terrain = g_shaders.compile("terrain");
	
	bool draw_terrain = true;
	//bool draw_water = true;

	bool dbg_lod = false;
		
	float lod_offset = 16;
	float lod_fac    = 64;

	int terrain_base_lod = -2;
	int water_base_lod = -1;

	int max_lod = 10;

	void imgui () {
		if (imgui_Header("TerrainRenderer")) {

			ImGui::Checkbox("dbg_lod", &dbg_lod);

			ImGui::Checkbox("draw_terrain", &draw_terrain);
			//ImGui::Checkbox("draw_water", &draw_water);

			ImGui::DragFloat("lod_offset", &lod_offset, 1, 0, 4096);
			ImGui::DragFloat("lod_fac", &lod_fac, 1, 0, 4096);

			ImGui::SliderInt("terrain_base_lod", &terrain_base_lod, -6, 6);
			ImGui::SliderInt("water_base_lod", &water_base_lod, -6, 6);

			ImGui::SliderInt("max_lod", &max_lod, -6, 15);

			ImGui::Text("drawn_chunks: %4d  (%7d vertices)", drawn_chunks, drawn_chunks * chunk_vertices);

			ImGui::PopID();
		}
	}

	TerrainRenderer () {
		gen_terrain_quad();
	}

	struct TerrainVertex {
		float2 pos;
		float3 offset_scale;
		float4 lod_bounds;

		VERTEX_CONFIG(
			ATTRIB(FLT,2, TerrainVertex, pos),
		)
	};
	struct TerrainChunkInstance {
		float3 offset_scale;
		float4 lod_bounds;

		VERTEX_CONFIG(
			ATTRIB(FLT,3, TerrainChunkInstance, offset_scale),
			ATTRIB(FLT,4, TerrainChunkInstance, lod_bounds),
		)
	};
	VertexBufferInstancedI vbo = vertex_buffer_instancedI<TerrainVertex, TerrainChunkInstance>("terrain");
	int chunk_vertices;
	int chunk_indices;
	
	int drawn_chunks = 0;

	template <typename FUNC>
	void lodded_chunks (StateManager& state, Heightmap& heightmap, Textures& texs,
			View3D& view, int base_lod, bool dbg, FUNC draw_chunk) {
		ZoneScoped;

		float2 lod_center = (float2)view.cam_pos;
		// TODO: adjust for terrain height? -> abs distance to full heightmap range might be reasonable
		// -> finding correct "distance" to heightmap terrain is somewhat impossible
		//    there could always be cases where a chunk has too high of a LOD compared to the distance the surface
		//    actually ends up being to the camera due to the heightmap
		float lod_center_z = view.cam_pos.z;

		int2 prev_bound0 = 0;
		int2 prev_bound1 = 0;

		int2 half_map_sz = heightmap.outer.map_size/2;
		
		// iterate lods
		for (int lod=base_lod; lod<=max_lod; lod++) {
			ZoneScopedN("lod");

			// size of this lod version of a chunk
			int sz = lod >= 0 ? TERRAIN_CHUNK_SZ << lod : TERRAIN_CHUNK_SZ >> (-lod);

			// parent (next higher res) lod chunk size
			int parent_sz = sz << 1;
			// mask that removes floors to this chunk size
			int parent_mask = ~(parent_sz-1);
			
			bool final_lod = lod == max_lod ||
				sz >= max(heightmap.outer.map_size.x, heightmap.outer.map_size.y);

			float quad_size = (float)sz / (float)TERRAIN_CHUNK_SZ;

			int2 bound0 = -half_map_sz;
			int2 bound1 =  half_map_sz;

			if (!final_lod) {
				// lod radius formula
				float radius = lod_offset + lod_fac * quad_size;
				if (lod != max_lod && lod_center_z >= radius)
					continue;
				radius = sqrt(radius*radius - lod_center_z*lod_center_z); // intersection of sphere with <radius> and z=0 plane to apply lodding at z=0 plane

				// for this lod radius, get the chunk grid bounds, aligned such that the parent lod chunks still fit without holes
				bound0 =  floori(lod_center - radius) & parent_mask;
				bound1 = (floori(lod_center + radius) & parent_mask) + parent_sz;
			}

			assert(bound1.x > bound0.x && bound1.y > bound0.y); // needed for next check to work (on prev_bound)

			// make sure there is always one chunk of buffer between any lod, ie. lod2 does not border lod4 directly
			// this is usually the case anyway, but avoid edge cases
			// > needed for lod seam fix to work
			if (prev_bound0.x != prev_bound1.x) { // if prev_bound is actually valid
				bound0 = min(bound0, (prev_bound0 & parent_mask) - parent_sz);
				bound1 = max(bound1, (prev_bound1 & parent_mask) + parent_sz);
			}
			
			bound0 = clamp(bound0, -half_map_sz, half_map_sz);
			bound1 = clamp(bound1, -half_map_sz, half_map_sz);

			// interate over all chunks in bounds
			for (int y=bound0.y; y<bound1.y; y+=sz)
			for (int x=bound0.x; x<bound1.x; x+=sz) {
				// exclude chunks that are already covered by smaller LOD chunks
				if (  x >= prev_bound0.x && x < prev_bound1.x &&
					  y >= prev_bound0.y && y < prev_bound1.y )
					continue;
					
				if (dbg)
					g_dbgdraw.wire_quad(float3((float2)int2(x,y), 0), (float2)(float)sz, g_dbgdraw.COLS[wrap(lod, ARRLEN(g_dbgdraw.COLS))]);
				
				draw_chunk(bound0, bound1, quad_size, int2(x,y));

				drawn_chunks++;
			}

			prev_bound0 = bound0;
			prev_bound1 = bound1;
		}

		if (dbg) {
			g_dbgdraw.wire_quad(float3(0 - (float2)heightmap.inner.map_size*0.5f, 0), (float2)heightmap.inner.map_size, lrgba(0,0,0,1));
			g_dbgdraw.wire_quad(float3(0 - (float2)heightmap.outer.map_size*0.5f, 0), (float2)heightmap.outer.map_size, lrgba(0,0,0,1));
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

		vbo.upload_mesh(verts, indices);
		chunk_vertices = vert_count;
		chunk_indices  = idx_count;
	}
	void render_terrain (StateManager& state, Heightmap& heightmap, Textures& texs, View3D& view, bool shadow_pass=false) {
		ZoneScoped;
		OGL_TRACE("render_terrain");

		drawn_chunks = 0;

		if (draw_terrain && shad_terrain->prog) {
			
			std::vector<TerrainChunkInstance> instances;

			lodded_chunks(state, heightmap, texs, view, terrain_base_lod, dbg_lod,
			[&] (int2 bound0, int2 bound1, float quad_size, int2 offset) {
				instances.push_back({
					float3( (float2)offset, quad_size ),
					float4( (float)bound0.x, (float)bound0.y, (float)bound1.x, (float)bound1.y )
				});
			});


			PipelineState s;
			if (!shadow_pass) {
				s.depth_test = true;
				s.blend_enable = false;
				s.cull_face = false;
			}
			else {
				s.depth_test = true;
				s.blend_enable = false;
				s.cull_face = false; // for shadow rendering and just becaue it won't hurt
				s.depth_clamp = true;
			}
			state.set(s);

			glUseProgram(shad_terrain->prog);

			state.bind_textures(shad_terrain,
				texs.heightmap.textures() + TextureBinds{{
				{"terrain_diffuse", *texs.terrain_diffuse, texs.sampler_normal},

				{ "grid_tex", *texs.grid, texs.sampler_normal },
			}});
			
			shad_terrain->set_uniform("inv_map_size", 1.0f / (float2)heightmap.inner.map_size);
			shad_terrain->set_uniform("inv_outer_size", 1.0f / (float2)heightmap.outer.map_size);
			shad_terrain->set_uniform("height_min", heightmap.height_min);
			shad_terrain->set_uniform("height_range", heightmap.height_range);

			vbo.stream_instances(instances);
			glBindVertexArray(vbo.vao);
			glDrawElementsInstanced(GL_TRIANGLES, chunk_indices, GL_UNSIGNED_SHORT, (void*)0, (GLsizei)instances.size());
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
#if 0
struct SkyboxRenderer {
	
	Shader* shad = g_shaders.compile("skybox");

	struct Vertex {
		float3 pos;
		
		VERTEX_CONFIG(
			ATTRIB(FLT,3, Vertex, pos),
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
	void render_skybox_last (StateManager& state, Textures& texs) {
		ZoneScoped;
		OGL_TRACE("draw_skybox");
		
		if (!shad->prog) return;
		PipelineState s;
		s.depth_test   = true;
		s.depth_write  = false;
		s.blend_enable = false;
		state.set_no_override(s);

		glEnable(GL_DEPTH_CLAMP);

		glUseProgram(shad->prog);

		state.bind_textures(shad, {
			{ "clouds", texs.clouds, texs.sampler_normal }
		});
		
		glBindVertexArray(skybox.vao);

		glDrawElements(GL_TRIANGLES, ARRLEN(indices), GL_UNSIGNED_SHORT, (void*)0);

		glDisable(GL_DEPTH_CLAMP);
	}
};
#endif

template <typename ASSET_T>
struct AssetMeshes {
	typedef typename decltype(ASSET_T().mesh)::vert_t vert_t;
	typedef typename decltype(ASSET_T().mesh)::idx_t  idx_t;

	static constexpr uint32_t COMPUTE_GROUPSZ = 512;
	
	Vbo vbo = {"entities.mesh.vbo"};
	Ebo ebo = {"entities.mesh.ebo"};
	
	Ssbo ssbo_mesh_info = {"mesh_info"};
	Ssbo ssbo_mesh_lod_info = {"mesh_lod_info"};

	std::unordered_map<ASSET_T*, int> asset2mesh_id;

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

	//int3 sz = -1;
	//glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 0, &sz.x);
	//glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 1, &sz.y);
	//glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 2, &sz.z);
	//
	//int3 cnt = -1;
	//glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 0, &cnt.x);
	//glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 1, &cnt.y);
	//glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 2, &cnt.z);

	void upload_meshes (AssetCollection<ASSET_T> const& assets) {
		ZoneScoped;
		OGL_TRACE("upload meshes");

		std::vector<MeshInfo> mesh_infos;
		std::vector<MeshLodInfo> mesh_lod_infos;

		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
			
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

	int compute_indirect_cmds (Shader* shad, Vbo& MDI_vbo, uint32_t instance_count) {
		if (!shad->prog) return 0;
		OGL_TRACE("lod_cull compute");
		
		// resize/clear cmd buf
		glBindBuffer(GL_ARRAY_BUFFER, MDI_vbo);
		glBufferData(GL_ARRAY_BUFFER, instance_count * sizeof(glDrawElementsIndirectCommand), nullptr, GL_STREAM_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		
		// assume (GL_SHADER_STORAGE_BUFFER, SSBO_BINDING_ENTITIY_INSTANCES) is instance buffer
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, SSBO_BINDING_ENTITY_MDI, MDI_vbo);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, SSBO_BINDING_ENTITY_MESH_INFO, ssbo_mesh_info);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, SSBO_BINDING_ENTITY_LOD_INFO, ssbo_mesh_lod_info);

		glUseProgram(shad->prog);

		shad->set_uniform("instance_count", instance_count);

		uint32_t groups_x = (instance_count + COMPUTE_GROUPSZ-1) / COMPUTE_GROUPSZ;
		glDispatchCompute(groups_x, 1, 1);

		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, SSBO_BINDING_ENTITY_MDI, 0);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, SSBO_BINDING_ENTITY_MESH_INFO, 0);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, SSBO_BINDING_ENTITY_LOD_INFO, 0);

		glMemoryBarrier(GL_COMMAND_BARRIER_BIT);

		return instance_count; // resulting commands count
	}
};

// A bit of a horrible class based around variadic template and std::tuple to allow to split instance data into multiple vbos for cleaner updates of only the dynamic parts
template <typename ASSET_T, typename... INSTANCE_PARTS>
struct EntityRenderer {
	const char* const dbg_name;

	Shader* shad;
	Shader* shad_lod_cull;

	AssetMeshes<ASSET_T>& meshes;

	EntityRenderer (const char* dbg_name, AssetMeshes<ASSET_T>& meshes, const char* shad_name, const char* variant_name="_VARIANT"):
		dbg_name{dbg_name}, meshes{meshes} {

		shad = g_shaders.compile(shad_name, {{variant_name, "1"}}); // #define <variant_name> to select specific shader

		shad_lod_cull = g_shaders.compile("lod_cull", {{variant_name, "1"},
			{"GROUPSZ", prints("%d", AssetMeshes<ASSET_T>::COMPUTE_GROUPSZ)},
		}, { shader::COMPUTE_SHADER });

		setup_vao();
	}

	// Really complicated variadic/tuple system that allows to easily split instance data into seperate structs/vbos that can then be individually uploaded
	template <typename T>
	struct VboPart {
		Vbo vbo = {"entities.vbo"};
		
		void upload (std::vector<T> const& data, GLenum usage=GL_STATIC_DRAW) {
			stream_buffer(GL_ARRAY_BUFFER, vbo, data, usage);
		}

		void setup (int& idx) {
			glBindBuffer(GL_ARRAY_BUFFER, vbo);
			idx = setup_vao_attribs(T::attribs(), idx, 1);
		}
	};

	Vao vao;
	std::tuple<VboPart<INSTANCE_PARTS>...> vbos;

	uint32_t instance_count = 0;

	template <int IDX, typename T>
	void upload (std::vector<T> const& data, bool streaming) {
		std::get<IDX>(vbos).upload(data, streaming ? GL_STREAM_DRAW : GL_STATIC_DRAW);

		if (IDX == 0) instance_count = (uint32_t)data.size();
		else assert(instance_count == (uint32_t)data.size());
	}

	void setup_vao () {
		vao = {"entities.vao"};
		vbos = {};

		glBindVertexArray(vao);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meshes.ebo);

		glBindBuffer(GL_ARRAY_BUFFER, meshes.vbo);
		int idx = setup_vao_attribs(AssetMeshes<ASSET_T>::vert_t::attribs(), 0, 0);

		// horrible syntax, essentially just loops of heterogenous elements in tuple vbos
		std::apply([&](auto&... vbos) {
			(vbos.setup(idx), ...);
		}, vbos);

		glBindBuffer(GL_ARRAY_BUFFER, 0); // glVertexAttribPointer remembers VAO
		glBindVertexArray(0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); // VAO remembers EBO (note that we need to unbind VAO first)
	}

	// All entities lodded draw commands
	Vbo MDI_vbo = {"DebugDraw.MDI_vbo"};
	
	void draw (StateManager& state, bool shadow_pass=false) {
		//ZoneScopedN(dbg_name); // TODO: why is this broken?
		ZoneScoped;
		OGL_TRACE("entities");
		
		auto& instance_vbo0 = std::get<0>(vbos).vbo;
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, SSBO_BINDING_ENTITY_INSTANCES, instance_vbo0);

		int cmds_count = meshes.compute_indirect_cmds(shad_lod_cull, MDI_vbo, instance_count);

		if (shad->prog) {
			OGL_TRACE("draw indirect");

			PipelineState s;
			if (!shadow_pass) {
				s.depth_test = true;
				s.blend_enable = false;
			}
			else {
				s.depth_test = true;
				s.blend_enable = false;
				s.depth_clamp = true;
			}
			state.set(s);

			glUseProgram(shad->prog);

			glBindVertexArray(vao);
			glBindBuffer(GL_DRAW_INDIRECT_BUFFER, MDI_vbo);

			state.bind_textures(shad, {});

			glMultiDrawElementsIndirect(GL_TRIANGLES, GL_UNSIGNED_SHORT, (void*)0, cmds_count, 0);
				
			glBindVertexArray(0);
			glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
		}

		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, SSBO_BINDING_ENTITY_INSTANCES, 0);

		//ImGui::Text("drawn vertices: %.3fM (indices: %.3fM)", drawn_vertices / 1000000.0f, drawn_indices / 1000000.0f); // TODO: ????? How to get this info?
	}
};

struct StaticEntity {
	int    mesh_id;
	int    tex_id;
	float3 pos;
	float  rot;
	float3 tint; // Just for debug?

	static constexpr const char* name = "StaticEntity";
	VERTEX_CONFIG(
		ATTRIB(INT,1, StaticEntity, mesh_id),
		ATTRIB(INT,1, StaticEntity, tex_id),
		ATTRIB(FLT,3, StaticEntity, pos),
		ATTRIB(FLT,1, StaticEntity, rot),
		ATTRIB(FLT,3, StaticEntity, tint),
	)
};
struct DynamicTrafficSignal {
	float3 colors[3];
	
	static constexpr const char* name = "DynamicTrafficSignal";
	VERTEX_CONFIG(
		ATTRIB(FLT,3, DynamicTrafficSignal, colors[0]),
		ATTRIB(FLT,3, DynamicTrafficSignal, colors[1]),
		ATTRIB(FLT,3, DynamicTrafficSignal, colors[2]),
	)
};
struct DynamicVehicle {
	int    mesh_id;
	int    instance_id; // uGH!!! avoiding this (gl_BaseInstance + gl_InstanceID) requires opengl 4.6
	int    tex_id;
	float3 pos;
	float3 tint;
	
	float _pad[3] = {};

	// can't be float3x4 even though that would be more efficient because that involves absurd hack where you load every float manually
	float4x4 bone_rot[5];

	static constexpr const char* name = "DynamicVehicle";
	VERTEX_CONFIG(
		ATTRIB(INT,1, DynamicVehicle, mesh_id),
		ATTRIB(INT,1, DynamicVehicle, instance_id),
		ATTRIB(INT,1, DynamicVehicle, tex_id),
		ATTRIB(FLT,3, DynamicVehicle, pos),
		ATTRIB(FLT,3, DynamicVehicle, tint),
	)
};

struct EntityRenderers {
	SERIALIZE(EntityRenderers, light_renderer)

	AssetMeshes<BuildingAsset> building_meshes;
	AssetMeshes<PropAsset>     prop_meshes;
	AssetMeshes<VehicleAsset>  vehicle_meshes;

	EntityRenderer<BuildingAsset, StaticEntity>                       buildings       = {"buildings",       building_meshes, "entities", "BUILDINGS"};
	EntityRenderer<PropAsset,     StaticEntity>                       props           = {"props",           prop_meshes,    "entities", "PROPS"};
	EntityRenderer<PropAsset,     StaticEntity>                       lamps           = {"lamps",           prop_meshes,    "entities", "LAMPS"};
	EntityRenderer<PropAsset,     StaticEntity, DynamicTrafficSignal> traffic_signals = {"traffic_signals", prop_meshes,    "entities", "TRAFFIC_SIGNALS"};
	EntityRenderer<VehicleAsset,  DynamicVehicle>                     vehicles        = {"vehicles",        vehicle_meshes, "vehicles", "VEHICLES"};

	DefferedPointLightRenderer light_renderer;

	void upload_meshes (Assets& assets) {
		building_meshes.upload_meshes(assets.buildings);
		vehicle_meshes .upload_meshes(assets.vehicles);
		prop_meshes    .upload_meshes(assets.props);
	}

	void draw_all (StateManager& state, bool shadow_pass=false) {
		buildings      .draw(state, shadow_pass);
		props          .draw(state, shadow_pass);
		lamps          .draw(state, shadow_pass);
		traffic_signals.draw(state, shadow_pass);
		vehicles       .draw(state, shadow_pass);
	}

	struct StaticInstances {
		EntityRenderers& rend;
		Textures& textures;

		std::vector<StaticEntity> buildings;
		std::vector<StaticEntity> props;
		std::vector<StaticEntity> lamps;
		std::vector<StaticEntity> traffic_signals;

		std::vector<DefferedPointLightRenderer::LightInstance> lights;

		void reserve () {
			buildings.reserve(4096);
			props.reserve(4096);
			lamps.reserve(4096);
			traffic_signals.reserve(512);
			lights.reserve(4096);
		}

		
		void _push_light (float3x4 const& matrix, PointLight& light) {
			auto* inst = push_back(lights, 1);
			
			inst->pos = matrix * light.pos;
			inst->radius = light.radius;
			inst->dir = (float3x3)matrix * light.dir;
			inst->cone.x = cos(light.cone.x);
			inst->cone.y = cos(light.cone.y);
			inst->col = light.col * light.strength;
		}
		void _push_base_prop (std::vector<StaticEntity>& vec, PropAsset* prop, float3 pos, float rot) {
			auto* inst = push_back(vec, 1);

			inst->mesh_id = rend.prop_meshes.asset2mesh_id[prop];
			inst->tex_id = textures.bindless_textures[prop->tex_filename];
			inst->pos = pos;
			inst->rot = rot;
			inst->tint = 1;

			auto matrix = obj_transform(pos, rot);
			for (auto& light : prop->lights) {
				_push_light(matrix, light);
			}
		}
		void push_prop (PropAsset* prop, float3 pos, float rot) {
			_push_base_prop(props, prop, pos, rot);
		}
		void push_lamp (PropAsset* prop, float3 pos, float rot) {
			_push_base_prop(lamps, prop, pos, rot);
		}
		void push_traffic_signal (PropAsset* prop, float3 pos, float rot) {
			_push_base_prop(traffic_signals, prop, pos, rot);
		}
	};
	void upload_static_instances (StaticInstances const& instances) {
		ZoneScoped;
		OGL_TRACE("upload_static_instances");
		
		buildings      .upload<0>(instances.buildings, false);
		props          .upload<0>(instances.props, false);
		lamps          .upload<0>(instances.lamps, false);
		traffic_signals.upload<0>(instances.traffic_signals, false);

		light_renderer.update_instances(instances.lights);
	}
};

struct NetworkRenderer {
	struct Vertex {
		float3 pos;
		float3 norm;
		float3 tang;
		float2 uv; // only really for some parts of the mesh, like curbs
		int    tex_id;
		//float4 col;

		VERTEX_CONFIG(
			ATTRIB(FLT,3, Vertex, pos),
			ATTRIB(FLT,3, Vertex, norm),
			ATTRIB(FLT,3, Vertex, tang),
			ATTRIB(FLT,2, Vertex, uv),
			ATTRIB(INT,1, Vertex, tex_id),
		)
	};

	Shader* shad  = g_shaders.compile("networks");

	VertexBufferI vbo = vertex_bufferI<Vertex>("NetworkRenderer.vbo");

	GLsizei indices_count = 0;

	void upload (Mesh<Vertex, uint32_t>& mesh) {
		vbo.upload(mesh.verticies, mesh.indices);
		indices_count = (GLsizei)mesh.indices.size();
	}
	
	void render (StateManager& state, Textures& texs, bool shadow_pass=false) {
		OGL_TRACE("network");
		ZoneScoped;

		if (shad->prog) {
			glUseProgram(shad->prog);

			state.bind_textures(shad, {

			});

			PipelineState s;
			if (!shadow_pass) {
				// default
			}
			else {
				s.depth_test = true;
				s.blend_enable = false;
				s.depth_clamp = true;
			}
			state.set(s);

			if (indices_count > 0) {
				glBindVertexArray(vbo.vao);
				glDrawElements(GL_TRIANGLES, indices_count, GL_UNSIGNED_INT, (void*)0);
			}
		}

		glBindVertexArray(0);
	}
};

struct Mesher {
	EntityRenderers& entity_renders;
	NetworkRenderer& network_render;
	DecalRenderer&   decal_render;
	ClippingRenderer& clip_render;
	App            & app;
	Textures       & textures;
	
	EntityRenderers::StaticInstances static_inst;

	Mesher (EntityRenderers& entity_renderers,
	        NetworkRenderer& network_renderer,
	        DecalRenderer& decal_renderer,
	        ClippingRenderer& clip_render,
	        App            & app,
			Textures       & textures):
		entity_renders{entity_renderers}, network_render{network_renderer}, decal_render{decal_renderer}, clip_render{clip_render}, app{app}, textures{textures},
		static_inst{ entity_renderers, textures } {}
	
	Mesh<NetworkRenderer::Vertex, uint32_t> network_mesh;
	std::vector<DecalRenderer::Instance> decals;
	std::vector<ClippingRenderer::Instance> clippings;
	
	void place_segment_line_props (network::Segment& seg, NetworkAsset::Streetlight& light) {
		float y = 0;
		while (y < seg._length) {
			// TODO: introduce curved segments, then rework this!
			float3 dir = seg.node_b->pos - seg.node_a->pos;
			float3 forw = normalizesafe(dir);
			float3 right = float3(rotate90(-(float2)forw), 0); // cw rotate
			float3 up = cross(right, forw);
			float rot = light.rot + angle2d(forw);
			float3 shift = light.shift + float3(0,y,0);
			float3 pos = seg.pos_a + right * shift.x + forw * shift.y + up * shift.z;

			static_inst.push_lamp(light.prop.get(), pos, rot);

			y += light.spacing;
		}
	}
	void place_traffic_light (network::Segment& seg, network::Node* node) {
		auto* prop = seg.asset->traffic_light_prop.get();

		// TODO: don't place mask if no incoming lanes (eg. because of one way road)
		
		auto* other_node = seg.get_other_node(node);
		float3 dir = node->pos - other_node->pos;
		float3 forw = normalizesafe(dir);
		float3 right = float3(rotate90(-(float2)forw), 0); // cw rotate
		float3 up = cross(right, forw);
		float rot = deg(90.0f) + angle2d(forw);

		float2 shift = seg.asset->traffic_light_shift;
		float3 pos = seg.pos_for_node(node) + right * shift.x + forw * shift.y;

		static_inst.push_prop(prop->mast_prop.get(), pos, rot);

		for (auto in_lane : seg.in_lanes(node)) {
			auto& layout = seg.get_lane_layout(&in_lane.get());
			float x = (seg.get_dir_to_node(node) == LaneDir::FORWARD ? +layout.shift : -layout.shift) - shift.x;

			float3 local_pos = prop->get_signal_pos(x);
			float3 spos = pos + right * local_pos.x + forw * local_pos.y + up * local_pos.z;

			static_inst.push_traffic_signal(prop->signal_prop.get(), spos, rot);
		}
	}

	// Could be instanced in smart ways, by computing beziers on gpu
	// for ex. for road segments: store 'cross section' per road time and extrude it along bezier during instancing on gpu
	// for intersections: greatly depends on how intersecyions will look, but could probably do similar
	// but if properly lodded (via chunks probably) should run fine and unlikely take too much vram

	// TODO: make sure to do proper indexing?
	// OR actually load road mesh as asset and morpth it into shape

	// TODO: fix tangents being wrong sometimes?

	float curbstone_w = 0.25f;

	// negative numbers are for non uv mappes (worldspace) textures
	int asphalt_tex_id;
	int curb_tex_id;
	int sidewalk_tex_id;

	float2 curb_tex_tiling = float2(2,0);

	float streetlight_spacing = 10;
	
	float stopline_width  = 1.0f;

	static constexpr float2 no_uv = 0;

	float3 norm_up = float3(0,0,1);
	float3 tang_up = float3(1,0,0);

	void mesh_segment (network::Segment& seg) {
		float width = seg.asset->width;
		
		float3 dir = seg.node_b->pos - seg.node_a->pos;

		float3 forw = normalizesafe(dir);
		float3 right = float3(rotate90(-(float2)forw), 0); // cw rotate
		float3 up = cross(right, forw);

		float3 tang = forw;
		
		typedef NetworkRenderer::Vertex V;
		
		float road_z = network::ROAD_Z;
		float sidw_z = 0;

		auto extrude = [&] (V const& l, V const& r, float2 uv_tiling=0) {
			V l0 = l;
			V l1 = l;
			V r0 = r;
			V r1 = r;

			l0.pos = seg.pos_a + right * l.pos.x + forw * l.pos.y + up * l.pos.z;
			l1.pos = seg.pos_b + right * l.pos.x + forw * l.pos.y + up * l.pos.z;

			r0.pos = seg.pos_a + right * r.pos.x + forw * r.pos.y + up * r.pos.z;
			r1.pos = seg.pos_b + right * r.pos.x + forw * r.pos.y + up * r.pos.z;

			if (uv_tiling.x > 0) {
				l1.uv.x += seg._length / uv_tiling.x;
				r1.uv.x += seg._length / uv_tiling.x;
			}
			if (uv_tiling.y > 0) {
				l1.uv.y += seg._length / uv_tiling.y;
				r1.uv.y += seg._length / uv_tiling.y;
			}

			network_mesh.push_quad(l0, r0, r1, l1);
		};
		
		float3 diag_right = (up + right) * SQRT_2/2;
		float3 diag_left  = (up - right) * SQRT_2/2;

		V sL0  = { float3(         -width*0.5f              , 0, sidw_z), up,         tang, no_uv, sidewalk_tex_id };
		V sL1  = { float3(seg.asset->sidewalkL - curbstone_w, 0, sidw_z), up,         tang, no_uv, sidewalk_tex_id };
		V sL1b = { float3(seg.asset->sidewalkL - curbstone_w, 0, sidw_z), up,         tang, float2(0,1), curb_tex_id };
		V sL2  = { float3(seg.asset->sidewalkL              , 0, sidw_z), diag_right, tang, float2(0,0.4f), curb_tex_id };
		V sL3  = { float3(seg.asset->sidewalkL              , 0, road_z), right,      tang, float2(0,0), curb_tex_id };
		V r0   = { float3(seg.asset->sidewalkL              , 0, road_z), up, tang, no_uv, asphalt_tex_id };
		V r1   = { float3(seg.asset->sidewalkR              , 0, road_z), up, tang, no_uv, asphalt_tex_id };
		V sR0  = { float3(seg.asset->sidewalkR              , 0, road_z), -right,    tang, float2(0,0), curb_tex_id };
		V sR1  = { float3(seg.asset->sidewalkR              , 0, sidw_z), diag_left, tang, float2(0,0.4f), curb_tex_id };
		V sR2b = { float3(seg.asset->sidewalkR + curbstone_w, 0, sidw_z), up,        tang, float2(0,1), curb_tex_id };
		V sR2  = { float3(seg.asset->sidewalkR + curbstone_w, 0, sidw_z), up,        tang, no_uv, sidewalk_tex_id };
		V sR3  = { float3(         +width*0.5f              , 0, sidw_z), up,        tang, no_uv, sidewalk_tex_id };

		extrude(sL0 , sL1 );
		extrude(sL1b, sL2 , curb_tex_tiling);
		extrude(sL2 , sL3 , curb_tex_tiling);
		extrude(r0  , r1  );
		extrude(sR0 , sR1 , curb_tex_tiling);
		extrude(sR1 , sR2b, curb_tex_tiling);
		extrude(sR2 , sR3 );
		
		for (auto& line : seg.asset->line_markings) {
			int tex_id = textures.bindless_textures[
				line.type == LineMarkingType::LINE ? "misc/line" : "misc/stripe"
			];
			
			float width = line.scale.x;

			float uv_len = seg._length / (line.scale.y*4); // *4 since 1-4 aspect ratio
			uv_len = max(round(uv_len), 1.0f); // round to avoid stopping in middle of stripe

			DecalRenderer::Instance decal;
			decal.pos = (seg.pos_a + seg.pos_b)*0.5f + float3(right * line.shift.x, 0);
			decal.rot = angle2d(forw) - deg(90);
			decal.size = float3(width, seg._length, 1);
			decal.tex_id = tex_id;
			decal.uv_scale = float2(1, uv_len);
			decal.col = 1;
			decals.push_back(decal);
		}

		for (auto& streetlight : seg.asset->streetlights) {
			place_segment_line_props(seg, streetlight);
		}
	}

	void mesh_node (network::Node* node) {
		struct SegInfo {
			float2 pos;
			float2 forw;
			float2 right;
			NetworkAsset* asset;
		};
		auto calc_seg_info = [&] (network::Segment* seg) {
			SegInfo info;

			auto* other = seg->get_other_node(node);
			info.forw = normalizesafe(node->pos - other->pos);
			info.right = rotate90(-info.forw);

			info.pos = seg->pos_for_node(node);

			info.asset = seg->asset;

			return info;
		};

		auto get_bez = [] (float2 a, float2 a_dir, float2 b, float2 b_dir) {
			network::Line la = { float3(a - a_dir,0), float3(a,0) };
			network::Line lb = { float3(b,0), float3(b - b_dir,0) };
			return network::calc_curve(la, lb);
		};

		int count = (int)node->segments.size();
		for (int i=0; i<count; ++i) {
			//network::Segment* l = node->segments[((i-1)+count) % count];
			network::Segment* s = node->segments[i];
			network::Segment* r = node->segments[(i+1) % count];

			auto si = calc_seg_info(s);
			auto ri = calc_seg_info(r);

			float2 sidewalk_Ra0  = si.pos + si.right * si.asset->sidewalkR;
			float2 sidewalk_Rb0  = si.pos + si.right * si.asset->width*0.5f;

			float2 sidewalk_Ra1  = ri.pos + ri.right * ri.asset->sidewalkL;
			float2 sidewalk_Rb1  = ri.pos - ri.right * ri.asset->width*0.5f;

			float2 sidewalk_RR = ri.pos + ri.right * ri.asset->sidewalkL;

			auto sidewalk_Ra = get_bez(sidewalk_Ra0, si.forw, sidewalk_Ra1, ri.forw);
			auto sidewalk_Rb = get_bez(sidewalk_Rb0, si.forw, sidewalk_Rb1, ri.forw);

			//sidewalk_Ra.dbg_draw(app.view, 0, 4, lrgba(1,0,0,1));
			//sidewalk_Rb.dbg_draw(app.view, 0, 4, lrgba(0,1,0,1));

			float2 segL = si.pos + si.right * si.asset->sidewalkL;
			float2 segR = si.pos + si.right * si.asset->sidewalkR;

			float road_z = network::ROAD_Z;
			float sidw_z = 0;

			typedef NetworkRenderer::Vertex V;
			V nodeCenter = { node->pos + float3(0, 0, road_z), norm_up, tang_up, no_uv, asphalt_tex_id };
			V seg0       = { float3(segL, road_z), norm_up, tang_up, no_uv, asphalt_tex_id };
			V seg1       = { float3(segR, road_z), norm_up, tang_up, no_uv, asphalt_tex_id };

			int res = 10;
			for (int i=0; i<res; ++i) {
				float t0 = (float)(i  ) / (res);
				float t1 = (float)(i+1) / (res);

				float2 a0 = sidewalk_Ra.eval(t0).pos;
				float2 a1 = sidewalk_Ra.eval(t1).pos;
				float2 b0 = sidewalk_Rb.eval(t0).pos;
				float2 b1 = sidewalk_Rb.eval(t1).pos;

				V sa0g = { float3(a0, road_z), norm_up, tang_up, no_uv, sidewalk_tex_id };
				V sa1g = { float3(a1, road_z), norm_up, tang_up, no_uv, sidewalk_tex_id };
				V sa0  = { float3(a0, sidw_z), norm_up, tang_up, no_uv, sidewalk_tex_id };
				V sa1  = { float3(a1, sidw_z), norm_up, tang_up, no_uv, sidewalk_tex_id };
				V sb0  = { float3(b0, sidw_z), norm_up, tang_up, no_uv, sidewalk_tex_id };
				V sb1  = { float3(b1, sidw_z), norm_up, tang_up, no_uv, sidewalk_tex_id };
				
				V sa0gA = { float3(a0, road_z), norm_up, tang_up, no_uv, asphalt_tex_id };
				V sa1gA = { float3(a1, road_z), norm_up, tang_up, no_uv, asphalt_tex_id };

				network_mesh.push_quad(sa0, sb0, sb1, sa1);
				network_mesh.push_quad(sa0g, sa0, sa1, sa1g);
				network_mesh.push_tri(sa0gA, sa1gA, nodeCenter);
			}

			network_mesh.push_tri(seg0, seg1, nodeCenter);
		}
		
		if (node->traffic_light) {
			for (auto& seg : node->segments) {
				place_traffic_light(*seg, node);
			}
		}
	}

	static void update_dynamic_traffic_signals (Network& net, EntityRenderers& entity_render) {
		std::vector<DynamicTrafficSignal> signal_colors;
		signal_colors.reserve(512);

		for (auto& node : net.nodes) {
			if (node->traffic_light) {
				assert(node->traffic_light->behavior);
				node->traffic_light->behavior->push_signal_colors(node.get(), signal_colors);
			}
		}

		entity_render.traffic_signals.upload<1>(signal_colors, true);
	}

	void remesh_network () {
		ZoneScoped;
		
		for (auto& seg : app.net.segments) {
			mesh_segment(*seg);

			auto v = seg->clac_seg_vecs();
			
			{
				float3 forw = normalizesafe(seg->pos_b - seg->pos_a);
				float ang = angle2d((float2)forw);

				float3 center = (seg->pos_a + seg->pos_b) * 0.5f;
				float3 size;
				size.x = distance(seg->pos_b, seg->pos_a) + 15.0f*2; // TODO: this will get done better with better road meshing
				size.y = seg->asset->width;
				size.z = 1.0f + 10.0f; // 10 meters above ground;
				float offs_z = -1.0f + size.z/2;

				ClippingRenderer::Instance clip;
				clip.pos = (seg->pos_a + seg->pos_b) * 0.5f;
				clip.pos.z += offs_z;
				clip.rot = ang;
				clip.size = size;
				clippings.push_back(clip);
			}

			for (laneid_t id=0; id<seg->num_lanes(); ++id) {
				auto seg_lane = network::SegLane{ seg.get(), id };
				auto& lane = seg->lanes[id];
				auto& lane_asset = seg->get_lane_layout(&lane);

				auto li = seg_lane.clac_lane_info();
				// TODO: this will be a bezier
				float3 forw = normalizesafe(li.b - li.a);
				//float3 right = float3(rotate90(-forw), 0);
				float ang = angle2d((float2)forw) - deg(90);

				{ // push turn arrow
					float2 size = float2(1, 1.5f) * lane_asset.width;

					float3 pos = li.b;
					pos -= forw * size.y*0.75f;

					auto filename = textures.turn_arrows[(int)lane.allowed_turns - 1];
					int tex_id = textures.bindless_textures[filename];

					DecalRenderer::Instance decal;
					decal.pos = pos;
					decal.rot = ang;
					decal.size = float3(size, 1);
					decal.tex_id = tex_id;
					decal.uv_scale = 1;
					decal.col = 1;
					decals.push_back(decal);
				}
				
				auto stop_line = [&] (float3 base_pos, float l, float r, int dir, bool type) {
					int tex_id = textures.bindless_textures[type ? "misc/shark_teeth" : "misc/line"];
					
					float width = type ? stopline_width*1.5f : stopline_width;
					float length = r - l;
					
					float uv_len = length / (width*2); // 2 since texture has 1-2 aspect ratio
					uv_len = max(round(uv_len), 1.0f); // round to avoid stopping in middle of stripe

					DecalRenderer::Instance decal;
					decal.pos = base_pos + float3(v.right * ((r+l)*0.5f), 0);
					decal.rot = angle2d(v.right) + deg(90) * (dir == 0 ? -1 : +1);
					decal.size = float3(width, length, 1);
					decal.tex_id = tex_id;
					decal.uv_scale = float2(1, uv_len);
					decal.col = 1;
					decals.push_back(decal);
				};

				if (lane_asset.direction == LaneDir::FORWARD) {
					float l = lane_asset.shift - lane_asset.width*0.5f;
					float r = lane_asset.shift + lane_asset.width*0.5f;

					stop_line(seg->pos_b, l, r, 0, lane.yield);
				}
				else {
					float l = lane_asset.shift - lane_asset.width*0.5f;
					float r = lane_asset.shift + lane_asset.width*0.5f;

					stop_line(seg->pos_a, l, r, 1, lane.yield);
				}
			}
		}

		for (auto& node : app.net.nodes) {
			mesh_node(node.get());
		}
	}

	void push_buildings () {
		ZoneScoped;

		static_inst.buildings.resize(app.entities.buildings.size());

		for (uint32_t i=0; i<(uint32_t)app.entities.buildings.size(); ++i) {
			auto& entity = app.entities.buildings[i];
			auto& inst = static_inst.buildings[i];
			
			inst.mesh_id = entity_renders.building_meshes.asset2mesh_id[entity->asset];
			inst.tex_id = textures.bindless_textures[entity->asset->tex_filename];
			inst.pos = entity->pos;
			inst.rot = entity->rot;
			inst.tint = 1;
		}
	}

	void remesh (App& app) {
		ZoneScoped;

		static_inst.reserve();
		
		// get diffuse texture id (normal is +1)
		// negative means worldspace uv mapped
		asphalt_tex_id  = -textures.bindless_textures[textures.asphalt];
		sidewalk_tex_id = -textures.bindless_textures[textures.pavement];
		curb_tex_id     =  textures.bindless_textures[textures.curb];

		push_buildings();

		remesh_network();
		
		entity_renders.upload_static_instances(static_inst);
		
		network_render.upload(network_mesh);
		decal_render.upload(decals);
		clip_render.upload(clippings);
	}
};

class OglRenderer : public Renderer {
	SERIALIZE(OglRenderer, lighting, passes, entity_render)
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
			
			passes.imgui();
			entity_render.light_renderer.imgui();
			textures.imgui();

		#if OGL_USE_REVERSE_DEPTH
			ImGui::Checkbox("reverse_depth", &ogl::reverse_depth);
		#endif

			gl_dbgdraw.imgui(g_dbgdraw.text);

			lod.imgui();

			lighting.imgui();
			terrain_render.imgui();
		}
		ImGui::End();
	}

	StateManager state;

	glDebugDraw gl_dbgdraw;
	
	struct Lighting {
		SERIALIZE(Lighting, sun_col, sky_col, fog_col, fog_base, fog_falloff)
		
		float time_of_day = 0.6f;
		float3 _pad0;
		
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

		void update (App& app, GameTime::SkyConfig& sky) {
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
	CommonUniforms common_ubo;
	
	RenderPasses passes;

	Lighting lighting;

	TerrainRenderer terrain_render;
	
	NetworkRenderer network_render;
	DecalRenderer   decal_render;

	ClippingRenderer clip_render;

	EntityRenderers entity_render;

	//SkyboxRenderer skybox;
	
	LOD_Func lod;

	Textures textures;

	OglRenderer () {
		
	}

	virtual void reload_textures () {
		textures.reload_all();
	}
	
	void upload_static_instances (App& app) {
		Mesher mesher{ entity_render, network_render, decal_render, clip_render, app, textures };
		mesher.remesh(app);
	}
	void upload_car_instances (App& app, View3D& view) {
		ZoneScoped;
		
		std::vector<DynamicVehicle> instances;
		instances.reserve(4096); // not all persons have active vehicle, don't overallocate

		for (auto& entity : app.entities.persons) {
			if (entity->vehicle) {
				uint32_t instance_idx = (uint32_t)instances.size();
				auto& instance = instances.emplace_back();

				update_vehicle_instance(instance, *entity, instance_idx, view);
			}
		}

		entity_render.vehicles.upload<0>(instances, true);
	}
	
	void update_vehicle_instance (DynamicVehicle& instance, Person& entity, int i, View3D& view) {
		auto& bone_mats = entity.owned_vehicle->bone_mats;
	
		int tex_id = textures.bindless_textures[entity.owned_vehicle->tex_filename];

		// TODO: network code shoud ensure length(dir) == CAR_SIZE
		float3 center;
		float ang;
		entity.vehicle->calc_pos(&center, &ang);

		instance.mesh_id = entity_render.vehicle_meshes.asset2mesh_id[entity.owned_vehicle];
		instance.instance_id = i;
		instance.tex_id = tex_id;
		instance.pos = center;
		instance.tint = entity.col;
				
		float3x3 heading_rot = rotate3_Z(ang);

		// skip expensive bone matricies computation when too far away
		float anim_LOD_dist = 250;
		if (length_sqr(center - view.cam_pos) > anim_LOD_dist*anim_LOD_dist) {
			for (auto& mat : instance.bone_rot) {
				mat = float4x4(heading_rot);
			}
			return;
		}

		// LOD this since currently bone matricies are too slow in debug mode
		// TODO: find a reasonable way to optimize this? Can't think of a clean way of doing matrix math much faster (especially in unoptimized mode)
		//    -> quaternions
		//  move to gpu? could easily do this in compute shader by passing wheel_roll, turn_curv, suspension_ang per instance as well
		//  could simply keep matricies in instance buffer and have compute shader read/write the vbo in different places? I think that's safe
		//  could also just split out the matrices into other buffer
		//  the question is if this is premature optimization, animated persons would be different again, and might want to move other code on gpu in the future
				
		auto set_bone_rot = [&] (int boneID, float3x3 const& bone_rot) {
			instance.bone_rot[boneID] = float4x4(heading_rot) *
				(bone_mats[boneID].bone2mesh * float4x4(bone_rot) * bone_mats[boneID].mesh2bone);
		};

		float wheel_ang = entity.vehicle->wheel_roll * -deg(360);
		float3x3 roll_mat = rotate3_Z(wheel_ang);

		float rear_axle_x = (bone_mats[VBONE_WHEEL_BL].bone2mesh * float4(0,0,0,1)).x;
				
		auto get_wheel_turn = [&] (int boneID) {
			// formula for ackerman steering with fixed rear axle
			// X is forw, Y is left
			float2 wheel_pos2d = (float2)(bone_mats[boneID].bone2mesh * float4(0,0,0,1));
			float2 wheel_rel2d = wheel_pos2d - float2(rear_axle_x, 0);

			float c = entity.vehicle->turn_curv;
			float ang = -atanf((c * wheel_rel2d.x) / (c * -wheel_rel2d.y - 1.0f));

			return rotate3_Y(ang);
		};

		float3x3 base_rot = rotate3_X(entity.vehicle->suspension_ang.x) * rotate3_Z(-entity.vehicle->suspension_ang.y);
		set_bone_rot(VBONE_BASE, base_rot);


		set_bone_rot(VBONE_WHEEL_FL, get_wheel_turn(VBONE_WHEEL_FL) * roll_mat);
		set_bone_rot(VBONE_WHEEL_FR, get_wheel_turn(VBONE_WHEEL_FR) * roll_mat);

		set_bone_rot(VBONE_WHEEL_BL, roll_mat);
		set_bone_rot(VBONE_WHEEL_BR, roll_mat);
	}
	
	virtual void begin (App& app) {
		ZoneScoped;
		gl_dbgdraw.gl_text_render.begin(g_dbgdraw.text); // upload text and init data structures to allow text printing
	}
	virtual void end (App& app, View3D& view) {
		ZoneScoped;
		
		auto sky_config = app.time.calc_sky_config(view);
		
		lighting.update(app, sky_config);
		
	#if RENDERER_DEBUG_LABELS
		// Dummy call because first gl event in nsight is always bugged, and by doing this the next OGL_TRACE() actually works
		glBindTexture(GL_TEXTURE_2D, 0);
	#endif
		
		{
			ZoneScopedN("uploads");
			OGL_TRACE("uploads");

			textures.heightmap.update_changes(app.heightmap);

			if (app.assets.assets_reloaded) {
				ZoneScopedN("assets_reloaded");
				entity_render.upload_meshes(app.assets);
			}

			if (app.entities.buildings_changed) {
				ZoneScopedN("buildings_changed");

				upload_static_instances(app);
			}

			upload_car_instances(app, view);
			Mesher::update_dynamic_traffic_signals(app.net, entity_render);
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

				glEnable(GL_LINE_SMOOTH);
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

		{
			ZoneScopedN("shadow_pass");
			OGL_TRACE("shadow_pass");

			passes.shadowmap.draw_cascades(view, sky_config, state, [&] (View3D& view, Fbo& depth_fbo, int2 res, GLenum depth_format) {
				common_ubo.set_view(view);
				
				terrain_render.render_terrain(state, app.heightmap, textures, view, true);
				clip_render.render(state, depth_fbo, res, depth_format, textures, true);
		
				network_render.render(state, textures, true);

				entity_render.draw_all(state, true);
			});
		}
		
		update_view_resolution(passes.renderscale.size);

		{
			ZoneScopedN("geometry_pass");
			OGL_TRACE("geometry_pass");
			
			passes.begin_geometry_pass(state);

			terrain_render.render_terrain(state, app.heightmap, textures, view);
			clip_render.render(state, passes.gbuf.fbo, passes.renderscale.size, passes.gbuf.depth_format, textures);

			network_render.render(state, textures);
			decal_render.render(state, passes.gbuf, textures);
		
			entity_render.draw_all(state);

			// TODO: draw during lighting pass?
			//  how to draw it without depth buffer? -> could use gbuf_normal == vec3(0) as draw condition?
			//skybox.render_skybox_last(state, textures);
		}

		{
			ZoneScopedN("lighting_pass");
			OGL_TRACE("lighting_pass");
			
			passes.begin_lighting_pass();

			passes.fullscreen_lighting_pass(state, textures, entity_render.light_renderer);

			passes.end_lighting_pass();
		}
		
		update_view_resolution(app.input.window_size);

		passes.postprocess(state, app.input.window_size);

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
