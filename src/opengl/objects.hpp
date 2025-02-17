#pragma once
#include "common.hpp"
#include "ogl_render.hpp"
#include "render_passes.hpp"

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

		return instance_count; // resulting commands count
	}
	void indirect_barrier () {
		glMemoryBarrier(GL_COMMAND_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT);
	}
};

// A bit of a horrible class based around variadic template and std::tuple to allow to split instance data into multiple vbos for cleaner updates of only the dynamic parts
template <typename ASSET_T, typename... INSTANCE_PARTS>
struct EntityRender {
	const char* const dbg_name;

	Shader* shad;
	Shader* shad_lod_cull;

	lrgba wireframe_col;

	AssetMeshes<ASSET_T>& meshes;
	
	EntityRender (const char* dbg_name, AssetMeshes<ASSET_T>& meshes, const char* shad_name, const char* variant_name="_VARIANT", lrgba wireframe_col={1}):
		dbg_name{dbg_name}, meshes{meshes}, wireframe_col{wireframe_col} {

		shad = g_shaders.compile(shad_name, {{variant_name, "1"}},
			false, prints("%s (%s)", shad_name, variant_name).c_str());

		shad_lod_cull = g_shaders.compile_compute("lod_cull", {{variant_name, "1"},
			{"GROUPSZ", prints("%d", AssetMeshes<ASSET_T>::COMPUTE_GROUPSZ)},
		}, false, prints("%s (%s)", "lod_cull", variant_name).c_str());

		setup_vao();
	}

	// Really complicated variadic/tuple system that allows to easily split instance data into seperate structs/vbos that can then be individually uploaded
	template <typename T>
	struct VboPart {
		Vbo vbo = {"entities.vbo"};
		
		void upload (std::vector<T> const& data, GLenum usage=GL_STATIC_DRAW) {
			upload_buffer(GL_ARRAY_BUFFER, vbo, data, usage);
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
	template <int IDX>
	void invalidate_instances () {
		glInvalidateBufferData(std::get<IDX>(vbos).vbo);
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
		ZoneScopedN(dbg_name);
		OGL_TRACE(dbg_name);
		
		auto& instance_vbo0 = std::get<0>(vbos).vbo;
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, SSBO_BINDING_ENTITY_INSTANCES, instance_vbo0);

		int cmds_count = meshes.compute_indirect_cmds(shad_lod_cull, MDI_vbo, instance_count);

		{
			OGL_TRACE("draw indirect");

			glUseProgram(shad->prog);

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

			shad->set_uniform("wireframe", state.wireframe && !shadow_pass);
			shad->set_uniform("wireframe_col", wireframe_col);

			glBindVertexArray(vao);
			glBindBuffer(GL_DRAW_INDIRECT_BUFFER, MDI_vbo);

			state.bind_textures(shad, {});

			meshes.indirect_barrier();
			glMultiDrawElementsIndirect(GL_TRIANGLES, GL_UNSIGNED_SHORT, (void*)0, cmds_count, 0);
				
			glBindVertexArray(0);
			glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
		}

		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, SSBO_BINDING_ENTITY_INSTANCES, 0);

		glInvalidateBufferData(MDI_vbo);

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
	float3 tint; // could be 8bit (srgb?)
	uint8v4 glow; // lights, brake lights, blinker L, blinker R; could probably be 4 bits!

	float _pad[2];

	// can't be float3x4 even though that would be more efficient because that involves absurd hack where you load every float manually
	float4x4 bone_rot[5];

	static constexpr const char* name = "DynamicVehicle";
	VERTEX_CONFIG(
		ATTRIB(INT,1, DynamicVehicle, mesh_id),
		ATTRIB(INT,1, DynamicVehicle, instance_id),
		ATTRIB(INT,1, DynamicVehicle, tex_id),
		ATTRIB(FLT,3, DynamicVehicle, pos),
		ATTRIB(FLT,3, DynamicVehicle, tint),
		ATTRIB(UBYTE_UNORM,4, DynamicVehicle, glow),
	)
};

struct EntityRenders {
	SERIALIZE(EntityRenders, lights)

	AssetMeshes<BuildingAsset> building_meshes;
	AssetMeshes<PropAsset>     prop_meshes;
	AssetMeshes<VehicleAsset>  vehicle_meshes;

	EntityRender<BuildingAsset, StaticEntity>                       buildings       = {"buildings",       building_meshes, "entities", "BUILDINGS",      lrgba(1,1,1, 1)};
	EntityRender<PropAsset,     StaticEntity>                       props           = {"props",           prop_meshes,    "entities", "PROPS",           lrgba(1,1,1, 1)};
	EntityRender<PropAsset,     StaticEntity>                       lamps           = {"lamps",           prop_meshes,    "entities", "LAMPS",           lrgba(1,1,1, 1)};
	EntityRender<PropAsset,     StaticEntity, DynamicTrafficSignal> traffic_signals = {"traffic_signals", prop_meshes,    "entities", "TRAFFIC_SIGNALS", lrgba(1,1,1, 1)};
	EntityRender<VehicleAsset,  DynamicVehicle>                     vehicles        = {"vehicles",        vehicle_meshes, "vehicles", "VEHICLES",        lrgba(1,0,0, 1)};

	DefferedPointLightRenderer lights;

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

		vehicles.invalidate_instances<0>();
	}
};

/* TODO: Rewrite entitiy rendering so that it can sort entities by assets, to test more traditional rendering technique for performance
* // After that implement version that allocates memory based on base mesh_id, such that cull/lod can be implmented with simple atomic add
instead of complicated compute sorting scheme, which hopefully is easy and runs efficently
class EntityRender2 {
public:

	struct Mesh {
		Vbo vbo = {"entities.mesh.vbo"};
		Ebo ebo = {"entities.mesh.ebo"};

		int index_count = 0;
	};
	std::unordered_map<void*, int> meshes;
	
	template <typename ASSET_T>
	void upload_meshes (AssetCollection<ASSET_T> const& assets) {
		ZoneScoped;

		for (auto& asset : assets) {
			auto& mesh = meshes[asset.get()];

			auto& lod0 = asset->mesh.mesh_lods[0];

			upload_buffer(GL_ARRAY_BUFFER        , mesh.vbo, lod0.vertices);
			upload_buffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo, lod0.indices);

			mesh.index_count = (int)lod0.indices.size();
		}
	}
	
	template <typename ASSET_T, typename... INSTANCE_PARTS>
	struct ShaderDraw {
		const char* const dbg_name;
		Shader* shader;

		lrgba wireframe_col;

		struct MeshDraw {
			Vao vao;

			std::vector<Vbo> vbos = {}; // "entities.vbo"

			int instance_count;
			
			void setup_vao (Mesh& mesh) {
				vao = {"entities.vao"};
				vbos = {};

				glBindVertexArray(vao);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo);

				glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
				int idx = setup_vao_attribs(AssetMeshes<ASSET_T>::vert_t::attribs(), 0, 0);

				// horrible syntax, essentially just loops of heterogenous elements in tuple vbos
				(setup_instance_part<INSTANCE_PARTS>(idx), ...);

				glBindBuffer(GL_ARRAY_BUFFER, 0); // glVertexAttribPointer remembers VAO
				glBindVertexArray(0);
				glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); // VAO remembers EBO (note that we need to unbind VAO first)
			}
			template <typename INSTANCE_T>
			void setup_instance_part (int& idx) {
				auto& vbo = vbos.emplace_back("entities.vbo");
				glBindBuffer(GL_ARRAY_BUFFER, vbo);
				idx = setup_vao_attribs(INSTANCE_T::attribs(), idx, 1);
			}
		};

		std::unordered_map<void*, MeshDraw> meshes;

		ShaderDraw (const char* dbg_name, const char* shad_name, const char* variant_name="_VARIANT", lrgba wireframe_col={1}):
				dbg_name{dbg_name}, wireframe_col{wireframe_col}{
			
			shader = g_shaders.compile(shad_name, {{variant_name, "1"}},
				false, prints("%s (%s)", shad_name, variant_name).c_str());
		}

		template <typename T>
		struct Data {
			std::vector<T> instances;

			T& push () {
				return instances.emplace_back();
			}
		};
		template <typename T>
		Data<T> prepare_data (int part) {
			Data<T> d;
			d.instances.reserve(512);
			return d;
		}

		void upload () {

		}

		void draw_all () {
			
		}
	};

	ShaderDraw<BuildingAsset, StaticEntity>                       buildings       = {"buildings",       "entities", "BUILDINGS",       lrgba(1,1,1, 1)};
	ShaderDraw<PropAsset,     StaticEntity>                       props           = {"props",           "entities", "PROPS",           lrgba(1,1,1, 1)};
	ShaderDraw<PropAsset,     StaticEntity>                       lamps           = {"lamps",           "entities", "LAMPS",           lrgba(1,1,1, 1)};
	ShaderDraw<PropAsset,     StaticEntity, DynamicTrafficSignal> traffic_signals = {"traffic_signals", "entities", "TRAFFIC_SIGNALS", lrgba(1,1,1, 1)};
	ShaderDraw<VehicleAsset,  DynamicVehicle>                     vehicles        = {"vehicles",        "vehicles", "VEHICLES",        lrgba(1,0,0, 1)};

	void upload_instance_data (Shader*) {

	}
};
*/

struct NetworkRender {
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
			// Render backfaces to avoid sun shining from below networks (due to terrain clipping) while setting
			// and shouldn't hurt performance that much usually, since networks mostly point towards sun
			s.cull_face = false;
		}
		state.set(s);

		shad->set_uniform("wireframe", state.wireframe && !shadow_pass);
		shad->set_uniform("wireframe_col", lrgba(1,1,1, 1));

		if (indices_count > 0) {
			glBindVertexArray(vbo.vao);
			glDrawElements(GL_TRIANGLES, indices_count, GL_UNSIGNED_INT, (void*)0);
			glBindVertexArray(0);
		}
	}
};

struct ObjectRender {
	SERIALIZE(ObjectRender, entities)

	EntityRenders entities;
	NetworkRender networks;

	DecalRenderer decals;
	CurvedDecalRender curved_decals;
	
	ClippingRenderer clippings;

	void imgui () {
		entities.lights.imgui();
	}

	void upload_static_instances (Textures& texs, App& app);
	void update_dynamic_traffic_signals (Textures& texs, Network& net);

	void upload_vehicle_instances (Textures& texs, App& app, View3D& view);

	void push_vehicle_instance (std::vector<DynamicVehicle>& instances,
		Textures& texs, Vehicle& veh, View3D& view, float dt);
	
	void push_parked_vehicle_instance (std::vector<DynamicVehicle>& instances,
		Textures& texs, Vehicle& veh);
	void push_vehicle_instance (std::vector<DynamicVehicle>& instances,
		Textures& texs, Vehicle& veh, network::SimVehicle& sim, View3D& view, float dt);
};

} // namespace ogl
