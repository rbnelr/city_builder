#include "common.hpp"
#include "assets.hpp"

#include "assimp/Importer.hpp"
#include "assimp/scene.h"
#include "assimp/postprocess.h"

// TODO: Why did I not use blender files directly?
// I think there might have been some issues with trying to do that?

namespace assimp {
	void print_scene (aiNode const* node, int depth, float4x4 const& transform) {
		auto& t = node->mTransformation;
		float4x4 mat = float4x4(t.a1,t.a2,t.a3,t.a4,  t.b1,t.b2,t.b3,t.b4, t.c1,t.c2,t.c3,t.c4,  t.d1,t.d2,t.d3,t.d4);
		mat = transform * mat;

		float3 pos = (float3)(mat * float4(0,0,0,1));

		for (int i=0; i<depth; ++i)
			printf("  ");
		printf("node: \"%s\" at (%f,%f,%f) Meshes: { ", node->mName.C_Str(), pos.x, pos.y, pos.z);

		for (unsigned int i=0; i<node->mNumMeshes; ++i) {
			printf("%d ", node->mMeshes[i]);
		}

		printf("}\n");

		for (unsigned int i=0; i<node->mNumChildren; ++i)
			print_scene(node->mChildren[i], depth+1, mat);
	}
	void print_scene (aiScene const* scene) {
		printf("FBX file: {\n");

		float scl = 1.0f / 100; // fbx seems to use centimeter units (with UnitScaleFactor 1) 

		for (unsigned int i=0; i<scene->mMetaData->mNumProperties; ++i) {
			if (strcmp(scene->mMetaData->mKeys[i].C_Str(), "UnitScaleFactor") == 0) {
				float fac = 1.0f;
				if (scene->mMetaData->mValues[i].mType == AI_DOUBLE) fac = (float)*(double*)scene->mMetaData->mValues[i].mData;
				if (scene->mMetaData->mValues[i].mType == AI_FLOAT)  fac =        *(float*) scene->mMetaData->mValues[i].mData;
				printf("  scene MetaData property UnitScaleFactor: %g\n", fac);
				scl *= fac;
			}
		}

		for (unsigned int i=0; i<scene->mNumMeshes; ++i) {
			printf("  mesh[%d]: \"%s\"\n", i, scene->mMeshes[i]->mName.C_Str());
		}

		print_scene(scene->mRootNode, 1, (float4x4)scale(float3(scl)));

		printf("}\n");
	}

	template <typename IDX_T>
	void push_face_indices (aiMesh const* mesh, std::vector<IDX_T>& indices, int first_vertex) {
		for (unsigned j=0; j<mesh->mNumFaces; ++j) {
			auto& f = mesh->mFaces[j];
			assert(f.mNumIndices == 3);

			for (unsigned k=0; k<3; ++k) {
				unsigned int idx = f.mIndices[k] + (unsigned int)first_vertex;
				assert(idx <= UINT16_MAX);
				indices.push_back( (uint16_t)idx );
			}
		}
	}

	void load_mesh_data (aiMesh const* mesh, float4x4 const& transform, Mesh<BasicVertex, uint16_t>& data, AABB3& aabb) {
		int first_vertex = (int)data.vertices.size();
		
		for (unsigned j=0; j<mesh->mNumVertices; ++j) {
			data.vertices.emplace_back();
			auto& v = data.vertices.back();

			auto& pos  = mesh->mVertices[j];
			auto& norm = mesh->mNormals[j];
			auto* uv   = &mesh->mTextureCoords[0][j];
			auto* col  = &mesh->mColors[0][j];

			v.pos    = (float3)(transform * float4(pos.x, pos.y, pos.z, 1.0f));
			v.normal = (float3)(transform * float4(norm.x, norm.y, norm.z, 0.0f));
			v.uv     = mesh->mTextureCoords[0] ? float2(uv->x, uv->y) : float2(0.0f);
			//v.col  = mesh->mColors[0] ? float4(col->r, col->g, col->b, col->a) : lrgba(1,1,1,1);

			aabb.add(v.pos);
		}

		push_face_indices(mesh, data.indices, first_vertex);
	}
	void load_mesh_data (aiMesh const* mesh, float4x4 const& transform, SimpleMesh& data) {
		int first_vertex = (int)data.vertices.size();
		
		for (unsigned j=0; j<mesh->mNumVertices; ++j) {
			data.vertices.emplace_back();
			auto& v = data.vertices.back();

			auto& pos  = mesh->mVertices[j];

			v.pos    = (float3)(transform * float4(pos.x, pos.y, pos.z, 1.0f));
		}

		push_face_indices(mesh, data.indices, first_vertex);
	}

	template <typename... ARGS>
	// Recursively load fbx tree, applying transformations to vertices
	void load_join_mesh_recurse (aiScene const* scene, aiNode const* node, float4x4 const& transform, ARGS&... args) {
		auto& t = node->mTransformation;
		float4x4 mat = float4x4(t.a1,t.a2,t.a3,t.a4,  t.b1,t.b2,t.b3,t.b4, t.c1,t.c2,t.c3,t.c4,  t.d1,t.d2,t.d3,t.d4);
		mat = transform * mat;
	
		for (unsigned int i=0; i<node->mNumMeshes; ++i) {
			unsigned int mesh_idx = node->mMeshes[i];
			assert(mesh_idx < scene->mNumMeshes);
			load_mesh_data(scene->mMeshes[mesh_idx], mat, args...);
		}
	
		for (unsigned int i=0; i<node->mNumChildren; ++i)
			load_join_mesh_recurse(scene, node->mChildren[i], mat, args...);
	}
	
	float4x4 get_base_transform (aiScene const* scene) {
		
		float scl = 1.0f / 100; // fbx seems to use centimeter units (with UnitScaleFactor 1) 
		for (unsigned int i=0; i<scene->mMetaData->mNumProperties; ++i) {
			if (strcmp(scene->mMetaData->mKeys[i].C_Str(), "UnitScaleFactor") == 0) {
				float fac = 1.0f;
				if (scene->mMetaData->mValues[i].mType == AI_DOUBLE) fac = (float)*(double*)scene->mMetaData->mValues[i].mData;
				if (scene->mMetaData->mValues[i].mType == AI_FLOAT)  fac =        *(float*) scene->mMetaData->mValues[i].mData;
				scl *= fac;
			}
		}

		return (float4x4)scale(float3(scl));
	}

	bool load (char const* filename, AssetMesh<BasicVertex>* out_mesh) {
		Assimp::Importer importer;

		auto* scene = importer.ReadFile(filename, aiProcess_Triangulate|aiProcess_JoinIdenticalVertices|aiProcess_CalcTangentSpace|aiProcess_ImproveCacheLocality);
		if (!scene) {
			fprintf(stderr, "Assimp error: %s\n", importer.GetErrorString());
			return false;
		}

		print_scene(scene);

		auto transform = get_base_transform(scene);
		
		if (!scene->mRootNode) return false;

		auto find_node = [&] (std::string_view tag) -> aiNode* {
			for (unsigned int i=0; i<scene->mRootNode->mNumChildren; ++i) {
				auto* node = scene->mRootNode->mChildren[i];
				if (contains(node->mName.C_Str(), tag))
					return node;
			}
			return nullptr;
		};

		for (int lod_i=0; lod_i<8; ++lod_i) {
			auto* node = find_node( prints("#lod%d", lod_i) );
			if (!node)
				break;
			out_mesh->mesh_lods.resize(lod_i+1);
			load_join_mesh_recurse(scene, node, transform, out_mesh->mesh_lods[lod_i], out_mesh->aabb);
		}

		return out_mesh->mesh_lods.size() > 0 &&
			out_mesh->mesh_lods[0].vertices.size() > 0 &&
			out_mesh->mesh_lods[0].indices.size() > 0;
	}
	
	bool load_simple (char const* filename, SimpleMesh* out_mesh) {
		Assimp::Importer importer;

		// Drop everything except vertex position to avoid aiProcess_JoinIdenticalVertices not producing optimal mesh
		importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, aiComponent_NORMALS | aiComponent_TANGENTS_AND_BITANGENTS | aiComponent_COLORS | aiComponent_TEXCOORDS |
			aiComponent_TEXTURES | aiComponent_LIGHTS | aiComponent_CAMERAS | aiComponent_MATERIALS);
		auto* scene = importer.ReadFile(filename, aiProcess_Triangulate|aiProcess_JoinIdenticalVertices|aiProcess_ImproveCacheLocality
			| aiProcess_DropNormals | aiProcess_RemoveComponent);
		if (!scene) {
			fprintf(stderr, "Assimp error: %s\n", importer.GetErrorString());
			return false;
		}

		print_scene(scene);

		auto transform = get_base_transform(scene);
		
		if (!scene->mRootNode) return false;

		for (unsigned int i=0; i<scene->mRootNode->mNumChildren; ++i) {
			auto* node = scene->mRootNode->mChildren[i];
			load_join_mesh_recurse(scene, node, transform, *out_mesh);
		}

		return out_mesh->vertices.size() > 0 &&
		       out_mesh->indices.size() > 0;
	}
}
