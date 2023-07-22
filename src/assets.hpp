#pragma once
#include "common.hpp"
#include "agnostic_render.hpp"
#include "kisslib/collision.hpp"
#include "engine/camera.hpp"

struct AssetMesh;

namespace assimp {
	bool load (char const* filename, AssetMesh* out_data);
}

// TODO: properly factor in base_size and custom lod curves into assets (especially if things like trees/rocks can be scaled)
// -> large buildings should only lod at very large distances (assuming their details are also large)
// -> also need to factor in fov -> calc visible resolution at x distance -> use this determine lod
// maybe also use bounding box distance for more accurate lod when close to large tower buildings?
struct LOD_Func {
	float bias = -3.0f;
	float fac  = 0.45f;

	void imgui () {
		if (ImGui::TreeNode("LOD")) {

			ImGui::DragFloat("bias", &bias, 0.1f);
			ImGui::DragFloat("fac", &fac, 0.1f, 0.0f);
			
			ImGui::TreePop();
		}
	}

	int pick_lod (View3D const& view, float3 const& obj_pos, float obj_lod_start) {
		float dist = max(distance(view.cam_pos, obj_pos) - obj_lod_start, 1.0f);

		int lod = roundi(fac * log2f(dist) + bias);
		lod = max(lod, 0);
		return lod;
	}
};

struct Mesh {
	struct Vertex {
		float3 pos;
		float3 normal;
		float2 uv;
		
		VERTEX_CONFIG(
			ATTRIB(FLT3, Vertex, pos),
			ATTRIB(FLT3, Vertex, normal),
			ATTRIB(FLT2, Vertex, uv),
		)
	};

	std::vector<Vertex> vertices;
	std::vector<uint16_t> indices;
};

struct AssetMesh {
	std::vector<Mesh> mesh_lods;

	AABB aabb;

	void recenter_xy () {
		float3 center = aabb.center();
		for (auto& m : mesh_lods) {
			for (auto& v : m.vertices) {
				v.pos.x -= center.x;
				v.pos.y -= center.y;
			}
		}
		aabb.lo -= center;
		aabb.hi -= center;
	}
	
	AssetMesh () {}

	AssetMesh (const char* filename) {
		assimp::load(prints("assets/%s", filename).c_str(), this);
		recenter_xy();
	}
};

struct BuildingAsset {
	friend SERIALIZE_TO_JSON(BuildingAsset) { SERIALIZE_TO_JSON_EXPAND(name, size) };

	std::string name;
	float3 size = 16;

	AssetMesh mesh;
};

struct Assets {
	friend SERIALIZE_TO_JSON(Assets) { SERIALIZE_TO_JSON_EXPAND(buildings); }
	friend SERIALIZE_FROM_JSON(Assets) {
		if (j.contains("buildings")) {
			for (auto& build : j.at("buildings")) {
				std::string name = build["name"];
				float3 size = build["size"];
				t.buildings.emplace_back(std::make_unique<BuildingAsset>(BuildingAsset{ name, size, prints("buildings/%s.fbx", name.c_str()).c_str() }));
			}
		}


		t.assets_reloaded = true;
	}

	bool assets_reloaded = true;

	// use a vector of pointers for now, asset pointers stay valid on edit, but need ordered data for gpu-side data
	template <typename T>
	using Collection = std::vector< std::unique_ptr<T> >;

	Collection<BuildingAsset> buildings;
};
