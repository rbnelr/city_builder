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
		ATTRIB(FLT3, BasicVertex, pos),
		ATTRIB(FLT3, BasicVertex, normal),
		ATTRIB(FLT2, BasicVertex, uv),
	)
};
struct VertexPos3 {
	float3 pos;

	VERTEX_CONFIG(
		ATTRIB(FLT3, VertexPos3, pos),
	)
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
	bool load (char const* filename, AssetMesh<BasicVertex>* out_data);
	bool load_simple (char const* filename, SimpleMesh* out_data);
}

template <typename VERT_T, typename IDX_T>
struct AssetMesh {
	typedef VERT_T vert_t;
	typedef IDX_T idx_t;
	
	std::vector< Mesh<VERT_T, IDX_T> > mesh_lods;

	AABB<float3> aabb;
	
	// needed so we can later load from json because json lib does not handle contructing from json
	AssetMesh () {}

	AssetMesh (const char* filename) {
		assimp::load(prints("assets/%s", filename).c_str(), this);
	}
};

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
	friend SERIALIZE_TO_JSON(NetworkAsset)   { SERIALIZE_TO_JSON_EXPAND(  name, filename, width, lanes, line_markings, streetlights, sidewalkL, sidewalkR, speed_limit) }
	friend SERIALIZE_FROM_JSON(NetworkAsset) { SERIALIZE_FROM_JSON_EXPAND(name, filename, width, lanes, line_markings, streetlights, sidewalkL, sidewalkR, speed_limit)
		//t.mesh = { prints("%s.fbx", t.filename.c_str()).c_str() };

		t.update_cached();
	}

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

		changed = imgui_edit_vector("lanes", lanes, [] (Lane& l) {
			bool changed = ImGui::DragFloat("shift", &l.shift, 0.1f);
			changed = ImGui::DragFloat("width", &l.width, 0.1f) || changed;

			int val = (int)l.direction;
			changed = ImGui::Combo("direction", &val, "forward\0backwards", 2) || changed;
			l.direction = (LaneDir)val;

			return changed;
		}) || changed;

		changed = imgui_edit_vector("line_markings", line_markings, [] (LineMarking& l) {
			bool changed = ImGui::Combo("type", (int*)&l.type, "LINE\0STRIPED", 2);
			changed = ImGui::DragFloat2("shift", &l.shift.x, 0.1f) || changed;
			changed = ImGui::DragFloat2("scale", &l.scale.x, 0.1f) || changed;
			return changed;
		}) || changed;

		changed = imgui_edit_vector("streetlights", streetlights, [] (Streetlight& l) {
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
};

// seperate into lists instead?
// Or maybe give buildings sub-buildings of certain types (maybe even track individual homes/workplaces?
enum class BuildingType {
	RESIDENTIAL,
	COMMERCIAL
};
NLOHMANN_JSON_SERIALIZE_ENUM(BuildingType, { { BuildingType::RESIDENTIAL, "RESIDENTIAL" }, { BuildingType::COMMERCIAL, "COMMERCIAL" } })

struct BuildingAsset {
	friend SERIALIZE_TO_JSON(BuildingAsset)   { SERIALIZE_TO_JSON_EXPAND(name, filename, type, citizens, size) }
	friend SERIALIZE_FROM_JSON(BuildingAsset) { SERIALIZE_FROM_JSON_EXPAND(name, filename, type, citizens, size)
		t.mesh = { t.filename.c_str() };
	}

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
};
struct CarAsset {
	friend SERIALIZE_TO_JSON(CarAsset)   { SERIALIZE_TO_JSON_EXPAND(name, filename) }
	friend SERIALIZE_FROM_JSON(CarAsset) { SERIALIZE_FROM_JSON_EXPAND(name, filename)
		t.mesh = { t.filename.c_str() };
	}

	std::string name = "<unnamed>";
	std::string filename;

	AssetMesh<BasicVertex> mesh;
};
struct PropAsset {
	friend SERIALIZE_TO_JSON(PropAsset)   { SERIALIZE_TO_JSON_EXPAND(name, filename) }
	friend SERIALIZE_FROM_JSON(PropAsset) { SERIALIZE_FROM_JSON_EXPAND(name, filename)
		t.mesh = { t.filename.c_str() };
	}

	std::string name = "<unnamed>";
	std::string filename;

	AssetMesh<BasicVertex> mesh;
};

struct Assets {
	friend SERIALIZE_TO_JSON(Assets) { SERIALIZE_TO_JSON_EXPAND(networks, buildings, cars, props) }
	friend SERIALIZE_FROM_JSON(Assets) { SERIALIZE_FROM_JSON_EXPAND(networks, buildings, cars, props)
		t.assets_reloaded = true;
	}

	bool assets_reloaded = true;

	// use a vector of pointers for now, asset pointers stay valid on edit, but need ordered data for gpu-side data
	template <typename T>
	using Collection = std::vector< std::unique_ptr<T> >;
	
	Collection<NetworkAsset>  networks;
	Collection<BuildingAsset> buildings;
	Collection<CarAsset>      cars;
	Collection<PropAsset>     props;
	
	Assets () {

	}

	void imgui (Settings& settings) {
		if (!ImGui::TreeNode("Assets")) return;
		
		imgui_edit_vector("networks", networks, [&] (std::unique_ptr<NetworkAsset>& network) {
			network->imgui(settings);
			return false;
		});
		imgui_edit_vector("buildings", buildings, [&] (std::unique_ptr<BuildingAsset>& building) {
			building->imgui(settings);
			return false;
		});

		ImGui::TreePop();
	}
};
