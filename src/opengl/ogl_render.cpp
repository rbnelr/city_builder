#pragma once
#include "common.hpp"
#include "app.hpp"
#include "engine/opengl.hpp"
#include "agnostic_render.hpp"
#include "gl_dbgdraw.hpp"
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
			ATTRIB(FLT,2, TerrainVertex, pos),
		)
	};
	VertexBufferI terrain_chunk = vertex_bufferI<TerrainVertex>("terrain");
	int chunk_vertices;
	int chunk_indices;
		
	int drawn_chunks = 0;

	template <typename FUNC>
	void lodded_chunks (StateManager& state, Textures& texs, View3D& view, Shader* shad, int base_lod, bool dbg, FUNC chunk) {
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
		chunk_indices  = idx_count;
	}
	void render_terrain (StateManager& state, Textures& texs, View3D& view, bool shadow_pass=false) {
		ZoneScoped;
		OGL_TRACE("render_terrain");

		drawn_chunks = 0;

		if (draw_terrain && shad_terrain->prog) {

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

			state.bind_textures(shad_terrain, {
			//	{"grid_tex", texs.grid, texs.sampler_normal},
			//	{"clouds", r.textures.clouds, r.textures.sampler_normal},
			//	{"heightmap", heightmap, r.sampler_heightmap},
				{"terrain_diffuse", texs.terrain_diffuse, texs.sampler_normal},
			});
			
			shad_terrain->set_uniform("inv_max_size", 1.0f / float2((float)MAP_SZ));

			glBindVertexArray(terrain_chunk.vao);
			
			lodded_chunks(state, texs, view, shad_terrain, terrain_base_lod, dbg_lod, [this] () {
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
		//	{ "clouds", r.textures.clouds, r.textures.sampler_normal }
		});
		
		glBindVertexArray(skybox.vao);

		glDrawElements(GL_TRIANGLES, ARRLEN(indices), GL_UNSIGNED_SHORT, (void*)0);

		glDisable(GL_DEPTH_CLAMP);
	}
};

struct StaticEntityInstance {
	int    mesh_id;
	float3 pos;
	float  rot;
	float3 col; // Just for debug?

	static constexpr const char* name = "StaticEntityInstance";
	
	VERTEX_CONFIG(
		ATTRIB(INT,1, StaticEntityInstance, mesh_id),
		ATTRIB(FLT,3, StaticEntityInstance, pos),
		ATTRIB(FLT,1, StaticEntityInstance, rot),
		ATTRIB(FLT,3, StaticEntityInstance, col),
	)
};
struct VehicleInstance {
	int    mesh_id;
	int    instance_id; // uGH!!! avoiding this (gl_BaseInstance + gl_InstanceID) requires opengl 4.6
	int    tex_id;
	float3 pos;
	float3 col; // Just for debug?
	
	float _pad[3] = {};

	// can't be float3x4 even though that would be more efficient because that involves absurd hack where you load every float manually
	float4x4 bone_rot[5];

	static constexpr const char* name = "VehicleInstance";

	VERTEX_CONFIG(
		ATTRIB(INT,1, VehicleInstance, mesh_id),
		ATTRIB(INT,1, VehicleInstance, instance_id),
		ATTRIB(INT,1, VehicleInstance, tex_id),
		ATTRIB(FLT,3, VehicleInstance, pos),
		ATTRIB(FLT,3, VehicleInstance, col),
	)
};

template <typename ASSET_T, typename INSTANCE_T>
struct EntityRenderer {
	typedef typename decltype(ASSET_T().mesh)::vert_t vert_t;
	typedef typename decltype(ASSET_T().mesh)::idx_t  idx_t;

	static constexpr uint32_t COMPUTE_GROUPSZ = 512;

	Shader* shad;
	Shader* shad_lod_cull = g_shaders.compile("lod_cull", {
			{"GROUPSZ", prints("%d", COMPUTE_GROUPSZ)},
			{"INSTANCE_T", INSTANCE_T::name},
		}, { shader::COMPUTE_SHADER });

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
	VertexBufferInstancedI vbo = vertex_buffer_instancedI<vert_t, INSTANCE_T>("meshes");
		
	Ssbo ssbo_mesh_info = {"mesh_info"};
	Ssbo ssbo_mesh_lod_info = {"mesh_lod_info"};

	// All entities lodded draw commands
	Vbo indirect_vbo = {"DebugDraw.indirect_draw"};

	std::unordered_map<ASSET_T*, int> asset2mesh_id;
		
	uint32_t entities_count = 0;
	
	EntityRenderer (const char* shad_name) {
		shad = g_shaders.compile(shad_name, {
			{"INSTANCE_T", INSTANCE_T::name},
		});
		shad_lod_cull = g_shaders.compile("lod_cull", {
			{"GROUPSZ", prints("%d", COMPUTE_GROUPSZ)},
			{"INSTANCE_T", INSTANCE_T::name},
		}, { shader::COMPUTE_SHADER });

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
	void update_instances (FUNC get_instances) {
		ZoneScoped;
		OGL_TRACE("update instances");

		auto instances = get_instances();
		vbo.stream_instances(instances);

		entities_count = (uint32_t)instances.size();
	}

	void draw (StateManager& state, Textures& texs, Texture2D& tex, bool shadow_pass=false, float time=0) {
		ZoneScoped;
		OGL_TRACE("draw entities");
		
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, vbo.instances);
		
		{
			OGL_TRACE("lod_cull compute");
				
			glBindBuffer(GL_ARRAY_BUFFER, indirect_vbo);
			glBufferData(GL_ARRAY_BUFFER, entities_count * sizeof(glDrawElementsIndirectCommand), nullptr, GL_STREAM_DRAW);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, indirect_vbo);
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, ssbo_mesh_info);
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, ssbo_mesh_lod_info);

			glUseProgram(shad_lod_cull->prog);

			shad_lod_cull->set_uniform("instances_count", entities_count);

			uint32_t groups_x = (entities_count + COMPUTE_GROUPSZ-1) / COMPUTE_GROUPSZ;
			glDispatchCompute(groups_x, 1, 1);
			glMemoryBarrier(GL_COMMAND_BARRIER_BIT);

			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, 0);
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, 0);
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, 0);
		}

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

			glBindVertexArray(vbo.vao);
			glBindBuffer(GL_DRAW_INDIRECT_BUFFER, indirect_vbo);

			state.bind_textures(shad, {
				{"tex", tex, texs.sampler_normal},
			});
			shad->set_uniform("time", time);

			glMultiDrawElementsIndirect(GL_TRIANGLES, GL_UNSIGNED_SHORT, (void*)0, entities_count, 0);
				
			glBindVertexArray(0);
			glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
		}
		
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, 0);

		//ImGui::Text("drawn vertices: %.3fM (indices: %.3fM)", drawn_vertices / 1000000.0f, drawn_indices / 1000000.0f); // TODO: ????? How to get this info?
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

	DecalRenderer decal_renderer;

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
	void render_decals (StateManager& state, Gbuffer& gbuf, Textures& texs) {
		OGL_TRACE("network decals");
		ZoneScoped;

		decal_renderer.render(state, gbuf, texs);
	}
};

struct Mesher {
	EntityRenderer<BuildingAsset, StaticEntityInstance>& building_renderer;
	NetworkRenderer                                    & network_renderer;
	EntityRenderer<PropAsset, StaticEntityInstance>    & prop_renderer;
	DefferedPointLightRenderer                         & light_renderer;
	Textures                                           & textures;
	
	std::vector<StaticEntityInstance> building_instances;
	std::vector<StaticEntityInstance> prop_instances;
	std::vector<DefferedPointLightRenderer::MeshInstance>    light_instances;
	
	Mesh<NetworkRenderer::Vertex, uint32_t> network_mesh;
	std::vector<DecalRenderer::Instance> decals;

	StaticEntityInstance& push_prop (PropAsset* asset, float3 pos, float rot) {
		auto* i = push_back(prop_instances, 1);
		
		i->mesh_id = prop_renderer.asset2mesh_id[asset];
		i->pos = pos;
		i->rot = rot;
		i->col = 1;

		return *i;
	}
	void push_light (float3 pos, float radius, lrgb col) {
		auto* i = push_back(light_instances, 1);
			
		i->pos = pos;
		i->radius = radius;
		i->col = col;
	}
	
	StaticEntityInstance& place_prop (network::Segment& seg, float3 shift, float rot, PropAsset* prop) {
		float3 dir = seg.node_b->pos - seg.node_a->pos;

		float3 forw = normalizesafe(dir);
		float3 right = float3(rotate90(-(float2)forw), 0); // cw rotate
		float3 up = cross(right, forw);
		
		float3 pos = seg.pos_a + right * shift.x + forw * shift.y + up * shift.z;
		rot += angle2d(forw);

		return push_prop(streetlight_asset, pos, rot);
	}


	// Could be instanced in smart ways, by computing beziers on gpu
	// for ex. for road segments: store 'cross section' per road time and extrude it along bezier during instancing on gpu
	// for intersections: greatly depends on how intersecyions will look, but could probably do similar
	// but if properly lodded (via chunks probably) should run fine and unlikely take too much vram

	// TODO: make sure to do proper indexing?
	// OR actually load road mesh as asset and morpth it into shape

	// TODO: fix tangents being wrong sometimes?

	PropAsset* streetlight_asset = nullptr;

	float sidewalk_h = 0.15f;
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
		
		auto extrude = [&] (V const& l, V const& r, float2 uv_tiling=0) {
			V l0 = l;
			V l1 = l;
			V r0 = r;
			V r1 = r;

			l0.pos = seg.pos_a + right * l.pos.x + forw * l.pos.y + up * l.pos.z + float3(0,0,0.01f);
			l1.pos = seg.pos_b + right * l.pos.x + forw * l.pos.y + up * l.pos.z + float3(0,0,0.01f);

			r0.pos = seg.pos_a + right * r.pos.x + forw * r.pos.y + up * r.pos.z + float3(0,0,0.01f);
			r1.pos = seg.pos_b + right * r.pos.x + forw * r.pos.y + up * r.pos.z + float3(0,0,0.01f);

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

		V sL0  = { float3(         -width*0.5f              , 0, sidewalk_h), up,         tang, no_uv, sidewalk_tex_id };
		V sL1  = { float3(seg.asset->sidewalkL - curbstone_w, 0, sidewalk_h), up,         tang, no_uv, sidewalk_tex_id };
		V sL1b = { float3(seg.asset->sidewalkL - curbstone_w, 0, sidewalk_h), up,         tang, float2(0,1), curb_tex_id };
		V sL2  = { float3(seg.asset->sidewalkL              , 0, sidewalk_h), diag_right, tang, float2(0,0.4f), curb_tex_id };
		V sL3  = { float3(seg.asset->sidewalkL              , 0,          0), right,      tang, float2(0,0), curb_tex_id };
		
		V r0   = { float3(seg.asset->sidewalkL, 0, 0), up, tang, no_uv, asphalt_tex_id };
		V r1   = { float3(seg.asset->sidewalkR, 0, 0), up, tang, no_uv, asphalt_tex_id };
		
		V sR0  = { float3(seg.asset->sidewalkR              , 0,          0), -right,    tang, float2(0,0), curb_tex_id };
		V sR1  = { float3(seg.asset->sidewalkR              , 0, sidewalk_h), diag_left, tang, float2(0,0.4f), curb_tex_id };
		V sR2b = { float3(seg.asset->sidewalkR + curbstone_w, 0, sidewalk_h), up,        tang, float2(0,1), curb_tex_id };
		V sR2  = { float3(seg.asset->sidewalkR + curbstone_w, 0, sidewalk_h), up,        tang, no_uv, sidewalk_tex_id };
		V sR3  = { float3(         +width*0.5f              , 0, sidewalk_h), up,        tang, no_uv, sidewalk_tex_id };

		extrude(sL0 , sL1 );
		extrude(sL1b, sL2 , curb_tex_tiling);
		extrude(sL2 , sL3 , curb_tex_tiling);
		extrude(r0  , r1  );
		extrude(sR0 , sR1 , curb_tex_tiling);
		extrude(sR1 , sR2b, curb_tex_tiling);
		extrude(sR2 , sR3 );
		
		for (auto& line : seg.asset->line_markings) {
			int tex_id = textures.bindless_textures.get_tex_id(
				line.type == LineMarkingType::LINE ? "misc/line.png" : "misc/stripe.png"
			);
			
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

		for (auto& light : seg.asset->streetlights) {
			float y = 0;
			while (y < seg._length) {
				auto& prop = place_prop(seg, light.shift + float3(0,y,0), light.rot, streetlight_asset);

				auto mat = obj_transform(prop.pos, prop.rot);
				auto light_pos = mat * light.light.pos;
				auto col = light.light.col * light.light.strength;

				push_light(light_pos, light.light.radius, col);

				y += light.spacing;
			}
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

			typedef NetworkRenderer::Vertex V;
			V nodeCenter = { node->pos + float3(0, 0, 0.01f), norm_up, tang_up, no_uv, asphalt_tex_id };
			V seg0       = { float3(segL, 0.01f), norm_up, tang_up, no_uv, asphalt_tex_id };
			V seg1       = { float3(segR, 0.01f), norm_up, tang_up, no_uv, asphalt_tex_id };

			int res = 10;
			for (int i=0; i<res; ++i) {
				float t0 = (float)(i  ) / (res);
				float t1 = (float)(i+1) / (res);

				float2 a0 = sidewalk_Ra.eval(t0).pos;
				float2 a1 = sidewalk_Ra.eval(t1).pos;
				float2 b0 = sidewalk_Rb.eval(t0).pos;
				float2 b1 = sidewalk_Rb.eval(t1).pos;

				V sa0g = { float3(a0,            + 0.01f), norm_up, tang_up, no_uv, sidewalk_tex_id };
				V sa1g = { float3(a1,            + 0.01f), norm_up, tang_up, no_uv, sidewalk_tex_id };
				V sa0  = { float3(a0, sidewalk_h + 0.01f), norm_up, tang_up, no_uv, sidewalk_tex_id };
				V sa1  = { float3(a1, sidewalk_h + 0.01f), norm_up, tang_up, no_uv, sidewalk_tex_id };
				V sb0  = { float3(b0, sidewalk_h + 0.01f), norm_up, tang_up, no_uv, sidewalk_tex_id };
				V sb1  = { float3(b1, sidewalk_h + 0.01f), norm_up, tang_up, no_uv, sidewalk_tex_id };
				
				V sa0gA = { float3(a0,            + 0.01f), norm_up, tang_up, no_uv, asphalt_tex_id };
				V sa1gA = { float3(a1,            + 0.01f), norm_up, tang_up, no_uv, asphalt_tex_id };

				network_mesh.push_quad(sa0, sb0, sb1, sa1);
				network_mesh.push_quad(sa0g, sa0, sa1, sa1g);
				network_mesh.push_tri(sa0gA, sa1gA, nodeCenter);
			}

			network_mesh.push_tri(seg0, seg1, nodeCenter);
		}
	}

	void remesh_network (network::Network& net) {
		ZoneScoped;
		
		for (auto& seg : net.segments) {
			mesh_segment(*seg);

			auto v = seg->clac_seg_vecs();
			
			for (int i=0; i<(int)seg->lanes.size(); ++i) {
				auto& lane = seg->lanes[i];
				network::SegLane seg_lane = { seg.get(), (uint16_t)i };

				auto li = seg_lane.clac_lane_info();
				// TODO: this will be a bezier
				float3 forw = normalizesafe(li.b - li.a);
				//float3 right = float3(rotate90(-forw), 0);
				float ang = angle2d((float2)forw) - deg(90);

				{ // push turn arrow
					float2 size = float2(1, 1.5f) * seg->asset->lanes[i].width;

					float3 pos = li.b;
					pos -= forw * size.y*0.75f;

					auto filename = textures.turn_arrows[(int)lane.allowed_turns - 1];
					int tex_id = textures.bindless_textures.get_tex_id(filename);

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
					int tex_id = textures.bindless_textures.get_tex_id(type ? "misc/shark_teeth.png" : "misc/line.png");
					
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


				auto& lane_asset = seg->asset->lanes[i];
				auto in_node = seg->get_node_in_dir(lane_asset.direction);
				auto& in_lane = in_node->get_in_lane(seg.get(), (uint16_t)i);

				if (lane_asset.direction == LaneDir::FORWARD) {
					float l = lane_asset.shift - lane_asset.width*0.5f;
					float r = lane_asset.shift + lane_asset.width*0.5f;

					stop_line(seg->pos_b, l, r, 0, in_lane.yield);
				}
				else {
					float l = lane_asset.shift - lane_asset.width*0.5f;
					float r = lane_asset.shift + lane_asset.width*0.5f;

					stop_line(seg->pos_a, l, r, 1, in_lane.yield);
				}
			}
		}

		for (auto& node : net.nodes) {
			mesh_node(node.get());
		}
	}
	void push_buildings (App& app) {
		ZoneScoped;

		building_instances.resize(app.entities.buildings.size());

		for (uint32_t i=0; i<(uint32_t)app.entities.buildings.size(); ++i) {
			auto& entity = app.entities.buildings[i];

			building_instances[i].mesh_id = building_renderer.asset2mesh_id[entity->asset];
			building_instances[i].pos = entity->pos;
			building_instances[i].rot = entity->rot;
			building_instances[i].col = 1;
		}

		building_renderer.update_instances([&] () { return building_instances; });
	}

	void remesh (App& app) {
		ZoneScoped;
		
		// get diffuse texture id (normal is +1)
		// negative means worldspace uv mapped
		asphalt_tex_id  = -textures.bindless_textures.get_tex_id(textures.asphalt.diffuse);
		sidewalk_tex_id = -textures.bindless_textures.get_tex_id(textures.pavement.diffuse);
		curb_tex_id     = textures.bindless_textures.get_tex_id(textures.curb.diffuse);

		remesh_network(app.net);

		push_buildings(app);

		prop_renderer.update_instances([&] () { return prop_instances; });
		light_renderer.update_instances([&] () { return light_instances; });
		
		network_renderer.upload(network_mesh);
		network_renderer.decal_renderer.upload(decals);
	}
};

static constexpr int COMMON_UBO_BINDING = 0;
static constexpr int BINDLESS_TEX_SSBO_BINDING = 1;
//static constexpr int DBGDRAW_INDIRECT_VBO = 2;

struct OglRenderer : public Renderer {
	SERIALIZE(OglRenderer, lighting, passes)
	
	virtual void to_json (nlohmann::ordered_json& j) {
		j["ogl_renderer"] = *this;
	}
	virtual void from_json (nlohmann::ordered_json const& j) {
		if (j.contains("ogl_renderer")) j.at("ogl_renderer").get_to(*this);
	}
	
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

		lrgb sun_col = lrgb(1.0f, 0.95f, 0.8f);
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
		Ubo ubo = {"common_ubo"};

		struct Common {
			View3D view;
			Lighting lighting;
		};

		void begin () {
			glBindBuffer(GL_UNIFORM_BUFFER, ubo);
			glBufferData(GL_UNIFORM_BUFFER, sizeof(Common), nullptr, GL_STREAM_DRAW);
			glBindBufferBase(GL_UNIFORM_BUFFER, COMMON_UBO_BINDING, ubo);
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
	struct BindlessTexLUT {

		Ssbo ssbo = {"bindless_ssbo"};

		void update (BindlessTextureManager& bindless) {
			std::vector<GLuint64> data;
			data.resize(bindless.loaded_textures.size());
			for (int i=0; i<(int)data.size(); ++i)
				data[i] = bindless.loaded_textures[i].tex.handle;

			glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
			glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(uint64_t)*data.size(), nullptr, GL_STREAM_DRAW);
			glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(uint64_t)*data.size(), data.data(), GL_STREAM_DRAW);
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, BINDLESS_TEX_SSBO_BINDING, ssbo);
			glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
		}
	};
	CommonUniforms common_ubo;
	BindlessTexLUT bindless_tex_lut;
	
	RenderPasses passes;

	Lighting lighting;

	TerrainRenderer terrain_renderer;
	
	NetworkRenderer network_renderer;

	EntityRenderer<BuildingAsset, StaticEntityInstance>   building_renderer{ "entities" };
	EntityRenderer<VehicleAsset,  VehicleInstance>        vehicle_renderer{  "vehicles" };
	// TODO: roll this into building renderer at least (car renderer might want different shader for wheel movement)
	// but doing so might require some problem solving to unify textures somehow
	// -> atlas or texture arrays for all texture sizes?
	EntityRenderer<PropAsset,     StaticEntityInstance>   prop_renderer{     "entities" };

	DefferedPointLightRenderer light_renderer;

	SkyboxRenderer skybox;
	
	LOD_Func lod;

	Textures textures;

	OglRenderer () {
		
	}
	
	void upload_static_instances (App& app) {
		Mesher mesher{ building_renderer, network_renderer, prop_renderer, light_renderer, textures };
		mesher.streetlight_asset = app.assets.props[0].get();

		mesher.remesh(app);
	}
	void upload_car_instances (App& app) {
		ZoneScoped;
		
		vehicle_renderer.update_instances([&] () {
			std::vector<VehicleInstance> instances;
			instances.resize(app.entities.citizens.size());

			for (uint32_t i=0; i<(uint32_t)app.entities.citizens.size(); ++i) {
				auto& entity = app.entities.citizens[i];
				if (!entity->agent) continue;

				auto& bone_mats = entity->owned_car->bone_mats;

				int tex_id = textures.bindless_textures.get_tex_id(entity->owned_car->tex_filename + ".diff.png");

				// TODO: network code shoud ensure length(dir) == CAR_SIZE
				float3 center;
				float ang;
				entity->agent->calc_pos(&center, &ang);

				instances[i].mesh_id = vehicle_renderer.asset2mesh_id[entity->owned_car];
				instances[i].instance_id = i;
				instances[i].tex_id = tex_id;
				instances[i].pos = center;
				instances[i].col = entity->col;
				
				float3x3 heading_rot = rotate3_Z(ang);
				auto set_bone_rot = [&] (int boneID, float3x3 const& bone_rot) {
					instances[i].bone_rot[boneID] = float4x4(heading_rot) *
						(bone_mats[boneID].bone2mesh * float4x4(bone_rot) * bone_mats[boneID].mesh2bone);
				};

				float wheel_ang = entity->agent->wheel_roll * -deg(360);
				float3x3 roll_mat = rotate3_Z(wheel_ang);

				float rear_axle_x = (bone_mats[VBONE_WHEEL_BL].bone2mesh * float4(0,0,0,1)).x;
				
				auto get_wheel_turn = [&] (int boneID) {
					// formula for ackerman steering with fixed rear axle
					// X is forw, Y is left
					float2 wheel_pos2d = (float2)(bone_mats[boneID].bone2mesh * float4(0,0,0,1));
					float2 wheel_rel2d = wheel_pos2d - float2(rear_axle_x, 0);

					float c = entity->agent->turn_curv;
					float ang = -atanf((c * wheel_rel2d.x) / (c * -wheel_rel2d.y - 1.0f));

					return rotate3_Y(ang);
				};

				float3x3 base_rot = rotate3_X(entity->agent->suspension_ang.x) * rotate3_Z(-entity->agent->suspension_ang.y);
				set_bone_rot(VBONE_BASE, base_rot);


				set_bone_rot(VBONE_WHEEL_FL, get_wheel_turn(VBONE_WHEEL_FL) * roll_mat);
				set_bone_rot(VBONE_WHEEL_FR, get_wheel_turn(VBONE_WHEEL_FR) * roll_mat);

				set_bone_rot(VBONE_WHEEL_BL, roll_mat);
				set_bone_rot(VBONE_WHEEL_BR, roll_mat);
			}

			return instances;
		});
	}

	virtual void begin (App& app) {
		ZoneScoped;
		gl_dbgdraw.gl_text_render.begin(g_dbgdraw.text); // upload text and init data structures to allow text printing
	}
	virtual void end (App& app, View3D& view) {
		ZoneScoped;

		lighting.sun_dir = float4(app.time_of_day.sun_dir, 0.0);

		if (app.assets.assets_reloaded) {
			ZoneScopedN("assets_reloaded");

			building_renderer.upload_meshes(app.assets.buildings);
			vehicle_renderer.upload_meshes(app.assets.vehicles);
			prop_renderer.upload_meshes(app.assets.props);
		}

		if (app.entities.buildings_changed) {
			ZoneScopedN("buildings_changed");

			upload_static_instances(app);
		}
		upload_car_instances(app);

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
				bindless_tex_lut.update(textures.bindless_textures);

				//glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gl_dbgdraw.indirect_vbo);

				gl_dbgdraw.update(app.input);
			}
		}

		passes.update(app.input.window_size);

		auto update_view_resolution = [&] (int2 res) {
			
			view.viewport_size = (float2)res;
			view.inv_viewport_size = 1.0f / view.viewport_size;

			common_ubo.set_view(view);
		};

		{
			ZoneScopedN("shadow_pass");
			OGL_TRACE("shadow_pass");

			passes.shadowmap.draw_cascades(app, view, state, [&] (View3D& view) {
				common_ubo.set_view(view);
				
				terrain_renderer.render_terrain(state, textures, view, true);
		
				network_renderer.render(state, textures, true);

				building_renderer.draw(state, textures, textures.house_diff, true);
				vehicle_renderer .draw(state, textures, textures.house_diff);
				prop_renderer    .draw(state, textures, textures.streetlight_diff, true);
			});
		}
		
		update_view_resolution(passes.renderscale.size);

		{
			ZoneScopedN("geometry_pass");
			OGL_TRACE("geometry_pass");
			
			passes.begin_geometry_pass(state);

			terrain_renderer.render_terrain(state, textures, view);

			network_renderer.render(state, textures);
			network_renderer.render_decals(state, passes.gbuf, textures);
		
			building_renderer.draw(state, textures, textures.house_diff);
			vehicle_renderer .draw(state, textures, textures.house_diff); // dummy tex, get rid of textures in general (building and props need bindless too)
			prop_renderer    .draw(state, textures, textures.streetlight_diff);

			// TODO: draw during lighting pass?
			//  how to draw it without depth buffer? -> could use gbuf_normal == vec3(0) as draw condition?
			skybox.render_skybox_last(state, textures);
		}

		{
			ZoneScopedN("lighting_pass");
			OGL_TRACE("lighting_pass");
			
			passes.begin_lighting_pass();

			passes.fullscreen_lighting_pass(state, textures, light_renderer);

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
