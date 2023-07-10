#pragma once
#include "common.hpp"
#include "agnostic_render.hpp"

struct LoadedMesh;
struct LoadedSubmesh;

namespace assimp {
	bool load_file_join (char const* filename, LoadedMesh* out_data, LoadedSubmesh* out_mesh);
}

struct LoadedSubmesh {
	int first_index;
	int index_count;
};
struct LoadedMesh {
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
	
	LoadedMesh () {}

	LoadedMesh (const char* filename) {
		LoadedSubmesh submesh;
		assimp::load_file_join(filename, this, &submesh);
	}
};

struct BuildingAsset {
	SERIALIZE(BuildingAsset, name, size)

	std::string name;
	float3 size = 16;

	LoadedMesh mesh;
	std::string text_filename;
};

struct Assets {
	SERIALIZE(Assets, buildings)

	template <typename T>
	using Collection = std::vector< std::unique_ptr<T> >;

	Collection<BuildingAsset> buildings;

	Assets () {
		buildings.push_back( std::make_unique<BuildingAsset>(BuildingAsset{"house", 16, "assets/house1/house.fbx", "assets/house1/house.png"}) );
	}
};
