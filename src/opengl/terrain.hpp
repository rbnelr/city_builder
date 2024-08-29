#pragma once
#include "common.hpp"
#include "ogl_common.hpp"
#include "heightmap.hpp"

namespace ogl {

struct TerrainRenderer {
	static constexpr int TERRAIN_CHUNK_SZ = 32; // 128 is max with GL_UNSIGNED_SHORT indices

	Shader* shad_terrain = g_shaders.compile("terrain");
	
	bool draw_terrain = true;

	bool dbg_lod = false;
		
	float lod_offset = 16;
	float lod_fac    = 64;

	int terrain_base_lod = 0;

	int max_lod = 10;

	void imgui () {
		if (imgui_Header("TerrainRenderer")) {

			ImGui::Checkbox("dbg_lod", &dbg_lod);

			ImGui::Checkbox("draw_terrain", &draw_terrain);

			ImGui::DragFloat("lod_offset", &lod_offset, 1, 0, 4096);
			ImGui::DragFloat("lod_fac", &lod_fac, 1, 0, 4096);

			ImGui::SliderInt("terrain_base_lod", &terrain_base_lod, -6, 6);

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

	// lod with camera position but cull with shadow views during shadow pass
	template <typename FUNC>
	void lodded_chunks (StateManager& state, Heightmap const& heightmap, Textures const& texs,
			View3D const& lodding_view, View3D const& culling_view, int base_lod, bool dbg, FUNC draw_chunk) {
		ZoneScoped;

		float2 lod_center = (float2)lodding_view.cam_pos;
		// TODO: adjust for terrain height? -> abs distance to full heightmap range might be reasonable
		// -> finding correct "distance" to heightmap terrain is somewhat impossible
		//    there could always be cases where a chunk has too high of a LOD compared to the distance the surface
		//    actually ends up being to the camera due to the heightmap
		float lod_center_z = lodding_view.cam_pos.z;

		int2 prev_bound0 = 0;
		int2 prev_bound1 = 0;

		int2 half_map_sz = heightmap.outer.map_size/2;

		auto frust = clac_view_frustrum(culling_view);
		
		float min_z = heightmap.height_min;
		float max_z = heightmap.height_min + heightmap.height_range;

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

				float2 pos = (float2)int2(x,y);
				
				auto aabb = AABB3(float3(pos, min_z), float3(pos + (float)sz, max_z));
				bool culled = frustrum_cull_aabb(frust, aabb);
				//culled = false;

				if (dbg) {
					lrgba col = g_dbgdraw.COLS[wrap(lod, ARRLEN(g_dbgdraw.COLS))];
					if (culled) col *= 0.25f;
					g_dbgdraw.wire_quad(float3(pos,0), (float2)(float)sz, col);
				}

				if (!culled) {
					draw_chunk(bound0, bound1, quad_size, int2(x,y));
					drawn_chunks++;
				}
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
	
	// view is not the actual rendering projection (eg for shadow pass, use camera view for lod/cull, not current shadow view)
	void render_terrain (StateManager& state, Heightmap const& heightmap, Textures const& texs,
			View3D const& lodding_view, View3D const& culling_view,
			bool shadow_pass=false) {
		ZoneScoped;
		OGL_TRACE("render_terrain");

		drawn_chunks = 0;

		if (draw_terrain && shad_terrain->prog) {
			
			std::vector<TerrainChunkInstance> instances;

			lodded_chunks(state, heightmap, texs, lodding_view, culling_view, terrain_base_lod, dbg_lod && !shadow_pass,
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
};

} // namespace ogl
