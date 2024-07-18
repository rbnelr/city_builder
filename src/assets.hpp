#pragma once
#include "common.hpp"
#include "agnostic_render.hpp"
#include "util.hpp"
#include "kisslib/collision.hpp"
#include "engine/camera.hpp"

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

struct BasicVertex {
	float3 pos;
	float3 normal;
	float2 uv;
		
	VERTEX_CONFIG(
		ATTRIB(FLT,3, BasicVertex, pos),
		ATTRIB(FLT,3, BasicVertex, normal),
		ATTRIB(FLT,2, BasicVertex, uv),
	)
};
// Mesh with anim bones but no interpolation between bones (single bone per vertex and no bone weights)
// -> used for vehicles
struct SimpleAnimVertex {
	float3  pos;
	float3  normal;
	float2  uv;
	uint8_t boneID;
	
	VERTEX_CONFIG(
		ATTRIB(FLT,3, SimpleAnimVertex, pos),
		ATTRIB(FLT,3, SimpleAnimVertex, normal),
		ATTRIB(FLT,2, SimpleAnimVertex, uv),
		ATTRIB(UBYTE,1, SimpleAnimVertex, boneID),
	)
};
struct VertexPos3 {
	float3 pos;

	VERTEX_CONFIG(
		ATTRIB(FLT,3, VertexPos3, pos),
	)
};

struct BoneMats {
	float4x4 mesh2bone = float4x4::identity();
	float4x4 bone2mesh = float4x4::identity();
};

inline float3x4 obj_transform (float3 pos, float rotZ) {
	float3x3 rot_mat = rotate3_Z(rotZ);
	
	//v.world_pos    = rot_mat * mesh_pos + instance_pos;
	return translate(pos) * rot_mat;
}

template <typename VERT_T, typename IDX_T>
struct Mesh {
	std::vector<VERT_T> vertices;
	std::vector<IDX_T> indices;
};

typedef Mesh<VertexPos3, uint16_t> SimpleMesh;

template <typename VERT_T, typename IDX_T=uint16_t> struct AssetMesh;

namespace assimp {
	bool load_basic (char const* filename, AssetMesh<BasicVertex, uint16_t>* out_mesh);
	bool load_simple_anim (char const* filename, AssetMesh<SimpleAnimVertex, uint16_t>* out_mesh,
		BoneMats* out_mats, float* out_wheel_r);

	bool load_simple (char const* filename, SimpleMesh* out_mesh);
}

template <typename VERT_T, typename IDX_T>
struct AssetMesh {
	typedef VERT_T vert_t;
	typedef IDX_T idx_t;
	
	std::vector< Mesh<VERT_T, IDX_T> > mesh_lods;

	AABB<float3> aabb;
};

#define SERIALIZE_ASSET(Type, ...) \
	friend SERIALIZE_TO_JSON(Type)   { SERIALIZE_TO_JSON_EXPAND(__VA_ARGS__) } \
	friend SERIALIZE_FROM_JSON(Type) { SERIALIZE_FROM_JSON_EXPAND(__VA_ARGS__) \
		t.reload(); \
	}

enum class LaneDir : uint8_t {
	FORWARD = 0,
	BACKWARD = 1,
};
NLOHMANN_JSON_SERIALIZE_ENUM(LaneDir, { { LaneDir::FORWARD, "FORWARD" }, { LaneDir::BACKWARD, "BACKWARD" } })

enum class LineMarkingType {
	LINE = 0,
	STRIPED = 1,
};
NLOHMANN_JSON_SERIALIZE_ENUM(LineMarkingType, { { LineMarkingType::LINE, "LINE" }, { LineMarkingType::STRIPED, "STRIPED" } })

struct PointLight {
	SERIALIZE(PointLight, pos, radius, col, strength)

	float3 pos = 0;
	float  radius = 5;
	lrgb   col = srgb(255, 246, 215);
	float  strength = 1;

	bool imgui (const char* name) {
		if (!ImGui::TreeNode(name)) return false;
		
		bool changed = ImGui::DragFloat3("pos", &pos.x, 0.1f);
		changed = ImGui::DragFloat("radius", &radius, 0.1f) || changed;
		changed = ImGui::ColorEdit3("col", &col.x, ImGuiColorEditFlags_DisplayHSV) || changed;
		changed = ImGui::DragFloat("strength", &strength, 0.1f) || changed;

		ImGui::TreePop();

		return changed;
	}
};

struct NetworkAsset {
	SERIALIZE_ASSET(NetworkAsset, name, filename, road_class, width, lanes, line_markings, streetlights, sidewalkL, sidewalkR, speed_limit)

	struct Lane {
		SERIALIZE(Lane, shift, width, direction)

		float shift = 0;
		float width = 3;
		LaneDir direction = LaneDir::FORWARD;
		// agent types (cars, trams, pedestrian etc.)
		
		// This is stupid
		// TODO: just sort lanes beforehand and return subsets as span?
		int order; // left to right lane index in direction
	};

	struct LineMarking {
		SERIALIZE(LineMarking, type, shift, scale)

		LineMarkingType type = LineMarkingType::LINE;
		float2 shift = 0;
		float2 scale = 1;
	};

	struct Streetlight {
		SERIALIZE(Streetlight, shift, spacing, rot, light)

		float3 shift = 0;
		float spacing = 10;
		float rot = deg(90);

		PointLight light;
	};
	
	std::string name = "<unnamed>";
	std::string filename;

	int road_class = 0;

	float width = 16;

	// for now: (RHD) outer forward, inner forward, ..., outer reverse, inner reverse
	// ie. sorted from left to right per direction
	// (relevant for pathing)
	std::vector<Lane> lanes;

	std::vector<LineMarking> line_markings;

	float sidewalkL = -6;
	float sidewalkR = +6;
	
	std::vector<Streetlight> streetlights;


	float speed_limit = 40 / KPH_PER_MS;

	// TODO: return list of all lanes in dir, how? span?
	// I just wish I had generator functions...
	int _lanes_in_dir[2];
	int num_lanes_in_dir (LaneDir dir) {
		return _lanes_in_dir[(int)dir];
	}


	void update_cached () {
		_lanes_in_dir[(int)LaneDir::FORWARD] = 0;
		_lanes_in_dir[(int)LaneDir::BACKWARD] = 0;

		for (auto& lane : lanes) {
			int& idx = _lanes_in_dir[(int)lane.direction];
			lane.order = idx;
			idx++;
		}
	}

	bool imgui (Settings& settings) {
		bool changed = false;

		changed = ImGui::InputText("name", &name) || changed;

		changed = ImGui::DragFloat("width", &width, 0.1f) || changed;

		changed = imgui_edit_vector("lanes", lanes, [] (int i, Lane& l) {
			bool changed = ImGui::DragFloat("shift", &l.shift, 0.1f);
			changed = ImGui::DragFloat("width", &l.width, 0.1f) || changed;

			int val = (int)l.direction;
			changed = ImGui::Combo("direction", &val, "forward\0backwards", 2) || changed;
			l.direction = (LaneDir)val;

			return changed;
		}) || changed;

		changed = imgui_edit_vector("line_markings", line_markings, [] (int i, LineMarking& l) {
			bool changed = ImGui::Combo("type", (int*)&l.type, "LINE\0STRIPED", 2);
			changed = ImGui::DragFloat2("shift", &l.shift.x, 0.1f) || changed;
			changed = ImGui::DragFloat2("scale", &l.scale.x, 0.1f) || changed;
			return changed;
		}) || changed;

		changed = imgui_edit_vector("streetlights", streetlights, [] (int i, Streetlight& l) {
			bool changed = ImGui::DragFloat3("shift", &l.shift.x, 0.1f);
			changed = ImGui::DragFloat("spacing", &l.spacing, 0.1f) || changed;
			changed = ImGui::SliderAngle("rot", &l.rot, 0, 360) || changed;
			changed = l.light.imgui("light") || changed;
			return changed;
		}) || changed;

		//changed = ImGui::Checkbox("has_sidewalk", &has_sidewalk) || changed;
		//if (has_sidewalk) {
			changed = ImGui::DragFloat("sidewalkL", &sidewalkL, 0.1f) || changed;
			changed = ImGui::DragFloat("sidewalkR", &sidewalkR, 0.1f) || changed;
		//}

		changed = imgui_slider_speed(settings, "speed_limit", &speed_limit, 0, 200/KPH_PER_MS) || changed;

		if (changed) update_cached();
		return changed;
	}

	void reload () {
		update_cached();
	}
};

// seperate into lists instead?
// Or maybe give buildings sub-buildings of certain types (maybe even track individual homes/workplaces?
enum class BuildingType {
	RESIDENTIAL,
	COMMERCIAL
};
NLOHMANN_JSON_SERIALIZE_ENUM(BuildingType, { { BuildingType::RESIDENTIAL, "RESIDENTIAL" }, { BuildingType::COMMERCIAL, "COMMERCIAL" } })

enum eVehicleBone {
	VBONE_BASE=0,
	VBONE_WHEEL_FL,
	VBONE_WHEEL_FR,
	VBONE_WHEEL_BL,
	VBONE_WHEEL_BR,

	VBONE_COUNT,
};
constexpr const char* VEHICLE_BONE_NAMES[] = {
	"Base",
	"Wheel.FL",
	"Wheel.FR",
	"Wheel.BL",
	"Wheel.BR",
};

struct BuildingAsset {
	SERIALIZE_ASSET(BuildingAsset, name, filename, type, citizens, size)

	std::string name = "<unnamed>";
	std::string filename;

	BuildingType type = BuildingType::RESIDENTIAL;
	int citizens = 10;

	float3 size = 16;
	
	AssetMesh<BasicVertex> mesh;

	bool imgui (Settings& settings) {
		bool changed = false;

		changed = ImGui::InputText("name", &name) || changed;
		
		changed = ImGui::Combo("type", (int*)&type, "RESIDENTIAL\0COMMERCIAL", 2) || changed;
		changed = ImGui::DragInt("citizens", &citizens, 0.1f) || changed;

		changed = ImGui::DragFloat3("size", &size.x, 0.1f) || changed;

		return changed;
	}

	void reload () {
		assimp::load_basic(prints("assets/%s", filename.c_str()).c_str(), &mesh);
	}
};
struct VehicleAsset {
	SERIALIZE_ASSET(VehicleAsset, name, filename, tex_filename, spawn_weight)

	std::string name = "<unnamed>";
	std::string filename;

	std::string tex_filename;

	float spawn_weight = 1;
	
	BoneMats bone_mats[5];

	float wheel_r = 0.5f;

	AssetMesh<SimpleAnimVertex> mesh;

	void reload () {
		assimp::load_simple_anim(prints("assets/%s", filename.c_str()).c_str(), &mesh, bone_mats, &wheel_r);
	}
};
struct PropAsset {
	SERIALIZE_ASSET(PropAsset, name, filename)

	std::string name = "<unnamed>";
	std::string filename;

	AssetMesh<BasicVertex> mesh;

	void reload () {
		assimp::load_basic(prints("assets/%s", filename.c_str()).c_str(), &mesh);
	}
};

// Asset loading scheme:
//  Assets class and each object is always constructed empty
//  debug.json contains all parameters for all asset objects
//  On app start json is loaded, this creates asset objects which call their reload() (through SERIALIZE_ASSET)
//  Each asset stores the data needed to reload without involving the json (eg. filenames, which usually would not be needed after load)
//   so that each object can be reloaded as desired (through UI, based on file changes or through json reload)
//  Assets class keeps assets_reloaded flag, and application has the responsibility of updating itself (stale asset pointers or stale data, meshes etc.)
//  NOTE: json load does not preserve anything inside containers, and any unique ptr is always recreated as well, so you always have to update anything using a asset ptr
//        but asset reload() does keep the asset ptr valid, but you might have to decide case by case what can stay, ex. lane changes in roads have to despawn cars etc.
//  TODO: the reload could possibly be handled better?, but probably fast enough for now, as it is mostly a dev thing

struct Assets {
	friend SERIALIZE_TO_JSON(Assets) { SERIALIZE_TO_JSON_EXPAND(networks, buildings, vehicles, props) }
	friend SERIALIZE_FROM_JSON(Assets) { SERIALIZE_FROM_JSON_EXPAND(networks, buildings, vehicles, props)
		t.assets_reloaded = true;
	}

	bool assets_reloaded = false;

	// use a vector of pointers for now, asset pointers stay valid on edit, but need ordered data for gpu-side data
	template <typename T>
	using Collection = std::vector< std::unique_ptr<T> >;
	
	Collection<NetworkAsset>  networks;
	Collection<BuildingAsset> buildings;
	Collection<VehicleAsset>  vehicles;
	Collection<PropAsset>     props;
	
	void reload_all () {
		for (auto& a : networks ) a->reload();
		for (auto& a : buildings) a->reload();
		for (auto& a : vehicles ) a->reload();
		for (auto& a : props    ) a->reload();
		assets_reloaded = true;
	}

	void imgui (Settings& settings) {
		if (!ImGui::TreeNode("Assets")) return;

		if (ImGui::Button("Reload All"))
			reload_all();
		
		imgui_edit_vector("networks", networks, [&] (int i, std::unique_ptr<NetworkAsset>& network) {
			network->imgui(settings);
			return false;
		});
		imgui_edit_vector("buildings", buildings, [&] (int i, std::unique_ptr<BuildingAsset>& building) {
			building->imgui(settings);
			return false;
		});

		ImGui::TreePop();
	}
};
