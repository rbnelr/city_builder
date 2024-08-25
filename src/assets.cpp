#include "common.hpp"
#include "assets.hpp"

#include "assimp/Importer.hpp"
#include "assimp/scene.h"
#include "assimp/postprocess.h"

// Don't actually applly transform matricies when loading vertex data, since simply using per-object origins is more flexible and faster to load as well
// this allows you to have a bunch of assets together in one blender file without having to clip them all into each other at the origin!
// Just make sure to not apply blender units when exporting, since this requires the base transform to be used

namespace assimp {
	float4x4 get_matrix (aiMatrix4x4t<float> const& m) {
		return float4x4(
			m.a1,m.a2,m.a3,m.a4,
			m.b1,m.b2,m.b3,m.b4,
			m.c1,m.c2,m.c3,m.c4,
			m.d1,m.d2,m.d3,m.d4
		);
	}
	float4x4 node_transform (aiNode const* node, float4x4 const& transform) {
		return transform * get_matrix(node->mTransformation);
	}

	uint8_t find_vertex_groupID (aiString const& mat_name) {
		for (int i=0; i<ARRLEN(VertexGroupID_Map); ++i) {
			if (kiss::contains(mat_name.C_Str(), VertexGroupID_Map[i].first))
				return VertexGroupID_Map[i].second;
		}
		return (uint8_t)0;
	}

	void print_scene (aiNode const* node, int depth, float4x4 const& transform) {
		float4x4 mat = node_transform(node, transform);

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
	void print_scene (aiScene const* scene, char const* filename) {
		printf("FBX file: %s {\n", filename);

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
		
		printf("Materials: {");
		for (unsigned int i=0; i<scene->mNumMaterials; ++i) {
			auto* mat = scene->mMaterials[i];
			auto name = mat->GetName();
			
			printf(" %s", name.C_Str());
		}
		printf(" }\n");

		for (unsigned int i=0; i<scene->mNumMeshes; ++i) {
			auto* m = scene->mMeshes[i];

			printf("  mesh[%d]: \"%s\" %d verts %d faces material: %d Bones: {\n", i, m->mName.C_Str(), m->mNumVertices, m->mNumFaces, m->mMaterialIndex);
			
			for (unsigned j=0; j<m->mNumBones; ++j) {
				auto& b = m->mBones[j];
				printf("    bone[%d]: \"%s\"\n", j, b->mName.C_Str());

				//auto mat = get_matrix(b->mOffsetMatrix);
				//printf("");
			}

			//for (unsigned j=0; j<m->mg; ++j) {
			//	auto& b = m->mBones[j];
			//	printf("    bone[%d]: \"%s\"\n", j, b->mName.C_Str());
			//
			//	//auto mat = get_matrix(b->mOffsetMatrix);
			//	//printf("");
			//}

			printf("  }\n");
		}

		print_scene(scene->mRootNode, 1, (float4x4)scale(float3(scl)));

		printf("}\n");
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

	template <typename IDX_T>
	void push_face_indices (aiMesh const* mesh, std::vector<IDX_T>& indices, unsigned base_vertex) {
		for (unsigned i=0; i<mesh->mNumFaces; ++i) {
			auto& f = mesh->mFaces[i];
			assert(f.mNumIndices == 3);

			for (unsigned k=0; k<3; ++k) {
				unsigned idx = f.mIndices[k] + (unsigned)base_vertex;
				assert(idx <= UINT16_MAX);
				indices.push_back( (uint16_t)idx );
			}
		}
	}
	
	void load_mesh_data (aiMesh const* mesh, aiMaterial* material, SimpleMesh<VertexPN>& data) {
		unsigned base_vertex = (unsigned)data.vertices.size();

		for (unsigned j=0; j<mesh->mNumVertices; ++j) {
			data.vertices.emplace_back();
			auto& v = data.vertices.back();

			auto& pos = mesh->mVertices[j];
			auto& norm = mesh->mNormals[j];

			v.pos = float3(pos.x, pos.y, pos.z);
			v.normal = float3(norm.x, norm.y, norm.z);
		}

		push_face_indices(mesh, data.indices, base_vertex);
	}
	void load_mesh_data (aiMesh const* mesh, aiMaterial* material, SimpleMesh<VertexPos3>& data) {
		unsigned base_vertex = (unsigned)data.vertices.size();
		
		for (unsigned j=0; j<mesh->mNumVertices; ++j) {
			data.vertices.emplace_back();
			auto& v = data.vertices.back();

			auto& pos = mesh->mVertices[j];

			v.pos = float3(pos.x, pos.y, pos.z);
		}

		push_face_indices(mesh, data.indices, base_vertex);
	}
	void load_mesh_data (aiMesh const* mesh, aiMaterial* material, Mesh<BasicVertex, uint16_t>& data, AABB3& aabb) {
		unsigned base_vertex = (unsigned)data.vertices.size();
		
		auto vtxGrpID = material ? find_vertex_groupID(material->GetName()) : (uint8_t)0;

		for (unsigned i=0; i<mesh->mNumVertices; ++i) {
			data.vertices.emplace_back();
			auto& v = data.vertices.back();

			auto& pos  = mesh->mVertices[i];
			auto& norm = mesh->mNormals[i];
			auto* uv   = &mesh->mTextureCoords[0][i];
			auto* col  = &mesh->mColors[0][i];

			v.pos    = float3(pos.x, pos.y, pos.z);
			v.normal = float3(norm.x, norm.y, norm.z);
			v.uv     = mesh->mTextureCoords[0] ? float2(uv->x, 1.0f-uv->y) : float2(0.0f);
			//v.col  = mesh->mColors[0] ? float4(col->r, col->g, col->b, col->a) : lrgba(1,1,1,1);
			v.vtxGrpID = vtxGrpID;

			aabb.add(v.pos);
		}

		push_face_indices(mesh, data.indices, base_vertex);
	}
	void load_mesh_data (aiMesh const* mesh, aiMaterial* material, Mesh<SimpleAnimVertex, uint16_t>& data, AABB3& aabb, std::unordered_map<int, int>& bone_id_map) {
		unsigned base_vertex = (unsigned)data.vertices.size();
		
		for (unsigned i=0; i<mesh->mNumVertices; ++i) {
			data.vertices.emplace_back();
			auto& v = data.vertices.back();

			auto& pos  = mesh->mVertices[i];
			auto& norm = mesh->mNormals[i];
			auto* uv   = &mesh->mTextureCoords[0][i];
			auto* col  = &mesh->mColors[0][i];

			v.pos    = float3(pos.x, pos.y, pos.z);
			v.normal = float3(norm.x, norm.y, norm.z);
			v.uv     = mesh->mTextureCoords[0] ? float2(uv->x, 1.0f-uv->y) : float2(0.0f);
			//v.col  = mesh->mColors[0] ? float4(col->r, col->g, col->b, col->a) : lrgba(1,1,1,1);
			v.boneID = 0; // assume first bone to get reasonable result at least // (uint8_t)-1;

			aabb.add(v.pos);
		}
		
		assert(mesh->mNumBones <= 255);
		for (unsigned bone_id=0; bone_id<mesh->mNumBones; ++bone_id) {
			auto* b = mesh->mBones[bone_id];
			uint8_t mapped_boneID = (uint8_t)bone_id_map.find((int)bone_id)->second;

			for (unsigned j=0; j<b->mNumWeights; ++j) {
				unsigned vert_id = b->mWeights[j].mVertexId;
				assert(vert_id >= 0 && vert_id < mesh->mNumVertices);
				if (b->mWeights[j].mWeight > 0.5f) {
					// need to map bone id from arbitrary blender ids to our chosen bone ids
					data.vertices[vert_id + base_vertex].boneID = mapped_boneID;
				}
			}
		}

		push_face_indices(mesh, data.indices, base_vertex);
	}

	template <typename... ARGS>
	// Recursively load fbx tree, applying transformations to vertices
	void load_join_mesh_recurse (aiScene const* scene, aiNode const* node, ARGS&... args) {
		for (unsigned int i=0; i<node->mNumMeshes; ++i) {
			assert(node->mMeshes[i] < scene->mNumMeshes);
			if (node->mMeshes[i] >= scene->mNumMeshes) continue; // for good measure
			auto* mesh = scene->mMeshes[node->mMeshes[i]];
			
			assert(mesh->mMaterialIndex < scene->mNumMaterials);
			auto* material = mesh->mMaterialIndex < scene->mNumMaterials ? scene->mMaterials[mesh->mMaterialIndex] : nullptr; // for good measure
			load_mesh_data(mesh, material, args...);
		}
	
		for (unsigned int i=0; i<node->mNumChildren; ++i)
			load_join_mesh_recurse(scene, node->mChildren[i], args...);
	}
	
	bool load_basic (char const* filename, AssetMesh<BasicVertex, uint16_t>* out_mesh) {
		printf("Loading basic mesh \"%s\"...\n", filename);
		ZoneScoped;

		Assimp::Importer importer;

		importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
			aiComponent_TEXTURES | aiComponent_LIGHTS | aiComponent_CAMERAS | aiComponent_MATERIALS |
			aiComponent_ANIMATIONS); // aiComponent_BONEWEIGHTS
		
		auto* scene = importer.ReadFile(filename, aiProcess_Triangulate|aiProcess_JoinIdenticalVertices|
			aiProcess_CalcTangentSpace|aiProcess_ImproveCacheLocality);
		if (!scene) {
			fprintf(stderr, "Assimp error: %s\n", importer.GetErrorString());
			return false;
		}

		//print_scene(scene, filename);
		
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
			load_join_mesh_recurse(scene, node, out_mesh->mesh_lods[lod_i], out_mesh->aabb);
		}

		return out_mesh->mesh_lods.size() > 0 &&
		       out_mesh->mesh_lods[0].vertices.size() > 0 &&
		       out_mesh->mesh_lods[0].indices.size() > 0;
	}

	bool load_simple_anim (char const* filename, AssetMesh<SimpleAnimVertex, uint16_t>* out_mesh,
			BoneMats* out_mats, float* out_wheel_r) {
		printf("Loading simple animated mesh \"%s\"...\n", filename);
		ZoneScoped;

		Assimp::Importer importer;

		importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
			aiComponent_TEXTURES | aiComponent_LIGHTS | aiComponent_CAMERAS | aiComponent_MATERIALS);
		
		auto* scene = importer.ReadFile(filename, aiProcess_Triangulate|aiProcess_JoinIdenticalVertices|
				aiProcess_CalcTangentSpace|aiProcess_ImproveCacheLocality|aiProcess_PopulateArmatureData);
		if (!scene) {
			fprintf(stderr, "Assimp error: %s\n", importer.GetErrorString());
			return false;
		}

		//print_scene(scene, filename);
		
		if (!scene->mRootNode) return false;

		std::unordered_map<int, int> bone_id_map;

		auto find_node = [&] (std::string_view tag) -> aiNode* {
			for (unsigned int i=0; i<scene->mRootNode->mNumChildren; ++i) {
				auto* node = scene->mRootNode->mChildren[i];
				if (contains(node->mName.C_Str(), tag))
					return node;
			}
			return nullptr;
		};
		auto find_bone = [&] (int bid) -> aiBone* {
			for (unsigned int i=0; i<scene->mNumMeshes; ++i) {
				auto* m = scene->mMeshes[i];
				for (unsigned j=0; j<m->mNumBones; ++j) {
					auto& b = m->mBones[j];
					if (contains(b->mName.C_Str(), VEHICLE_BONE_NAMES[bid])) {
						bone_id_map[j] = bid;
						return b;
					}
				}
			}
			return nullptr;
		};

		for (int bid=0; bid<VBONE_COUNT; ++bid) {
			auto* bone = find_bone(bid);
			if (bone) {
				out_mats[bid].mesh2bone = get_matrix(bone->mOffsetMatrix);
				out_mats[bid].bone2mesh = inverse(out_mats[bid].mesh2bone);

				if (bid == VBONE_WHEEL_BL) {
					auto* node = bone->mNode;
					if (node && node->mNumChildren >= 1) {
						// Get wheel radius from blender bone length
						float3 rel_pos = float3x4(get_matrix(node->mChildren[0]->mTransformation)) * float3(0,0,0);
						*out_wheel_r = length(rel_pos);
					}
					else {
						fprintf(stderr, "Vehicle WHEEL_BL Bone Child not found to get wheel radius!\n");
					}
				}
			}
			else {
				fprintf(stderr, "Vehicle Bone %s not found!\n", VEHICLE_BONE_NAMES[bid]);
			}
		}

		for (int lod_i=0; lod_i<8; ++lod_i) {
			auto* node = find_node( prints("#lod%d", lod_i) );
			if (!node)
				break;
			out_mesh->mesh_lods.resize(lod_i+1);
			load_join_mesh_recurse(scene, node, out_mesh->mesh_lods[lod_i], out_mesh->aabb, bone_id_map);
		}

		return out_mesh->mesh_lods.size() > 0 &&
		       out_mesh->mesh_lods[0].vertices.size() > 0 &&
		       out_mesh->mesh_lods[0].indices.size() > 0;
	}
	
	template<> bool load_simple<VertexPos3> (char const* filename, SimpleMesh<VertexPos3>* out_mesh) {
		printf("Loading simple mesh \"%s\"...\n", filename);
		ZoneScoped;

		Assimp::Importer importer;
		// Drop everything except vertex position to avoid aiProcess_JoinIdenticalVertices not producing optimal mesh
		importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
			aiComponent_NORMALS | aiComponent_TANGENTS_AND_BITANGENTS | aiComponent_COLORS | aiComponent_TEXCOORDS |
			aiComponent_BONEWEIGHTS | aiComponent_ANIMATIONS |
			aiComponent_TEXTURES | aiComponent_LIGHTS | aiComponent_CAMERAS | aiComponent_MATERIALS);

		auto* scene = importer.ReadFile(filename, aiProcess_Triangulate|aiProcess_JoinIdenticalVertices|
			aiProcess_ImproveCacheLocality|
			aiProcess_DropNormals|aiProcess_RemoveComponent);
		if (!scene) {
			fprintf(stderr, "Assimp error: %s\n", importer.GetErrorString());
			return false;
		}

		//print_scene(scene, filename);
		
		if (!scene->mRootNode) return false;
		load_join_mesh_recurse(scene, scene->mRootNode, *out_mesh);

		return out_mesh->vertices.size() > 0 && out_mesh->indices.size() > 0;
	}
	
	template<> bool load_simple<VertexPN> (char const* filename, SimpleMesh<VertexPN>* out_mesh) {
		printf("Loading simple mesh \"%s\"...\n", filename);
		ZoneScoped;

		Assimp::Importer importer;
		importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
			aiComponent_BONEWEIGHTS | aiComponent_ANIMATIONS |
			aiComponent_TEXTURES | aiComponent_LIGHTS | aiComponent_CAMERAS | aiComponent_MATERIALS);

		auto* scene = importer.ReadFile(filename, aiProcess_Triangulate|aiProcess_JoinIdenticalVertices|
			aiProcess_ImproveCacheLocality|
			aiProcess_RemoveComponent);
		if (!scene) {
			fprintf(stderr, "Assimp error: %s\n", importer.GetErrorString());
			return false;
		}

		//print_scene(scene, filename);
		
		if (!scene->mRootNode) return false;
		load_join_mesh_recurse(scene, scene->mRootNode, *out_mesh);

		return out_mesh->vertices.size() > 0 && out_mesh->indices.size() > 0;
	}
}
