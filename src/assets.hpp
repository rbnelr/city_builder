#pragma once
#include "common.hpp"
#include "textures.hpp"
#include "engine/camera.hpp"

struct Assets;

// TODO: properly factor in base_size and custom lod curves into assets (especially if things like trees/rocks can be scaled)
// -> large buildings should only lod at very large distances (assuming their details are also large)
// -> also need to factor in fov -> calc visible resolution at x distance -> use this determine lod
// maybe also use bounding box distance for more accurate lod when close to large tower buildings?
struct LOD_Func {
	float bias = -3.0f;
	float fac  = 0.45f;

	void imgui () {
		if (imgui_Header("LOD")) {

			ImGui::DragFloat("bias", &bias, 0.1f);
			ImGui::DragFloat("fac", &fac, 0.1f, 0.0f);
			
			ImGui::PopID();
		}
	}

	int pick_lod (View3D const& view, float3 const& obj_pos, float obj_lod_start) {
		float dist = max(distance(view.cam_pos, obj_pos) - obj_lod_start, 1.0f);

		int lod = roundi(fac * log2f(dist) + bias);
		lod = max(lod, 0);
		return lod;
	}
};

// Vertex shader for traffic lights, car blinkers etc. can find correct light data based on these IDs
// Specified in blender via assigning a material with special naming eg: "SomeMaterial.SignalYellow" and passed in mesh vertex data (similar to anim bone IDs)
// 0-> default behavoir, likely always albedo tinting based on (PBR.r * instance.tint)
// other numbers can be used to apply PBR.r as emmisive mask instead, use VertexGroupID to pull the appropriate color
inline constexpr std::pair<std::string_view, uint8_t> VertexGroupID_Map[] = {
	{"SignalRed",1}, {"SignalYellow",2}, {"SignalGreen",3}
};

struct BasicVertex {
	float3  pos;
	float3  normal;
	float2  uv;
	uint8_t vtxGrpID;
	
	VERTEX_CONFIG(
		ATTRIB(FLT,3,   BasicVertex, pos),
		ATTRIB(FLT,3,   BasicVertex, normal),
		ATTRIB(FLT,2,   BasicVertex, uv),
		ATTRIB(UBYTE,1, BasicVertex, vtxGrpID),
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
struct VertexPN {
	float3 pos;
	float3 normal;

	VERTEX_CONFIG(
		ATTRIB(FLT,3, VertexPN, pos),
		ATTRIB(FLT,3, VertexPN, normal),
	)
};

struct BoneMats {
	float4x4 mesh2bone = float4x4::identity();
	float4x4 bone2mesh = float4x4::identity();
};

inline float3x4 obj_transform (float3 pos, float rotZ) {
	float3x3 rot_mat = rotate3_Z(rotZ);
	return translate(pos) * rot_mat;
}

template <typename VERT_T, typename IDX_T>
struct Mesh {
	std::vector<VERT_T> vertices;
	std::vector<IDX_T> indices;
};

template <typename VERT>
using SimpleMesh = Mesh<VERT, uint16_t>;

template <typename VERT_T, typename IDX_T=uint16_t> struct AssetMesh;

namespace assimp {
	bool load_basic (char const* filename, AssetMesh<BasicVertex, uint16_t>* out_mesh);
	bool load_simple_anim (char const* filename, AssetMesh<SimpleAnimVertex, uint16_t>* out_mesh,
		BoneMats* out_mats, float* out_wheel_r);

	template <typename VERT>
	bool load_simple (char const* filename, SimpleMesh<VERT>* out_mesh);
}

template <typename VERT_T, typename IDX_T>
struct AssetMesh {
	typedef VERT_T vert_t;
	typedef IDX_T idx_t;
	
	std::vector< Mesh<VERT_T, IDX_T> > mesh_lods;

	AABB<float3> aabb;
};

////
struct Asset {
	std::string name = "<unnamed>";

	bool imgui () {
		return ImGui::InputText("name", &name);
	}
};

template <typename T> using AssetPtr = std::shared_ptr<T>;

template <typename T> void to_json (json& j, AssetPtr<T> const& ptr);
template <typename T> void from_json (json const& j, AssetPtr<T>& ptr);

////
enum class LaneDir : uint8_t {
	FORWARD = 0,
	BACKWARD = 1,
};
NLOHMANN_JSON_SERIALIZE_ENUM(LaneDir, { { LaneDir::FORWARD, "FORWARD" }, { LaneDir::BACKWARD, "BACKWARD" } })

typedef uint16_t laneid_t;

enum class LineMarkingType {
	LINE = 0,
	STRIPED = 1,
};
NLOHMANN_JSON_SERIALIZE_ENUM(LineMarkingType, { { LineMarkingType::LINE, "LINE" }, { LineMarkingType::STRIPED, "STRIPED" } })

template <typename T>
inline AssetPtr<T> dummy_asset () {
	static AssetPtr<T> asset = [] () {
		T a;
		a.name = "<dummy>";
		// TODO: ???
		return std::make_shared<T>(std::move(a));
	} ();
	return asset;
}

struct PointLight {
	SERIALIZE(PointLight, pos, dir, radius, cone, col, strength)

	float3 pos = 0;
	float  radius = 5;
	float3 dir = float3(0,0,-1); // TODO: Ideally a quaternion
	float2 cone = deg(360); // fade out start and ends in radiants
	lrgb   col = srgb(255, 246, 215);
	float  strength = 1;

	bool imgui () {
		bool changed = ImGui::DragFloat3("pos", &pos.x, 0.1f);
		changed = ImGui::DragFloat3("dir", &dir.x, 0.1f) || changed;
		changed = ImGui::DragFloat("radius", &radius, 0.1f) || changed;
		changed = ImGui::SliderAngle("spotlight inner cone", &cone.x, 0, 180) || changed;
		changed = ImGui::SliderAngle("spotlight outer cone", &cone.y, cone.x, 180) || changed;
		changed = ImGui::ColorEdit3("col", &col.x, ImGuiColorEditFlags_DisplayHSV) || changed;
		changed = ImGui::DragFloat("strength", &strength, 0.1f) || changed;

		if (changed)
			dir = normalizesafe(dir);

		return changed;
	}
};

struct PropAsset : public Asset {
	SERIALIZE(PropAsset, mesh_filename, tex_filename, lights)

	std::string mesh_filename;
	std::string tex_filename;

	AssetMesh<BasicVertex> mesh;

	// Optional lights for every prop!
	// TODO: decide if this makes sense in the end, currently it's now clear how these lights should be controlled
	//  controlled as in, turn off at night or not etc.
	std::vector<PointLight> lights;

	bool imgui (Options& opt) {
		bool changed = Asset::imgui();
		
		// TODO: implement generic vector/set iteration with item add/remove feature (sort by name for sets?)
		for (int i=0; i<(int)lights.size(); ++i) {
			if (ImGui::TreeNode(&lights[i], "[%d]", i)) {
				if (ImGui::TableNextColumn()) {
					changed = lights[i].imgui() || changed;
				}
				ImGui::TreePop();
			}
		}

		//changed = imgui_edit_vector("lights", lights, [&] (int i, PointLight& l) {
		//	return l.imgui("light");
		//}) || changed;

		return changed;
	}

	void reload () {
		mesh = {}; // need to clear mesh
		assimp::load_basic(prints("assets/%s", mesh_filename.c_str()).c_str(), &mesh);
	}
};
struct TrafficLightAsset : public Asset {
	SERIALIZE(TrafficLightAsset, mast_prop, signal_prop, mast_base, mast_dir, colors)

	AssetPtr<PropAsset> mast_prop = dummy_asset<PropAsset>();
	AssetPtr<PropAsset> signal_prop = dummy_asset<PropAsset>();

	float3 mast_base = float3(0,0,4);
	float3 mast_dir = float3(-1,0,0);
	
	lrgb colors[3] = {
		lrgb(1.0f,0.1f,0.1f),
		lrgb(1.0f,1.0f,0.1f),
		lrgb(0.1f,1.0f,0.1f),
	};

	float3 get_signal_pos (float x) {
		x -= mast_base.x;
		float t = x / mast_dir.x;
		return mast_base + mast_dir * t;
	}

	bool imgui (Options& opt) {
		bool changed = ImGui::DragFloat3("mast_base", &mast_base.x, 0.1f);
		changed = ImGui::DragFloat3("mast_dir", &mast_dir.x, 0.1f) || changed;
		mast_dir = normalizesafe(mast_dir);

		imgui_ColorEdit("Signal Red Color",    &colors[0]);
		imgui_ColorEdit("Signal Yellow Color", &colors[1]);
		imgui_ColorEdit("Signal Green Color",  &colors[2]);
		return changed;
	}

	void reload () {
		
	}
};

struct NetworkAsset : public Asset {
	SERIALIZE(NetworkAsset, road_class, width, sidewalkL, sidewalkR, lanes, line_markings, streetlights, traffic_light_prop, traffic_light_shift, sidewalkL, sidewalkR, speed_limit)

	struct Lane {
		SERIALIZE(Lane, shift, width, direction)

		float shift = 0;
		float width = 3;
		LaneDir direction = LaneDir::FORWARD;
		// user types (cars, trams, pedestrian etc.)
	};

	struct LineMarking {
		SERIALIZE(LineMarking, type, shift, scale)

		LineMarkingType type = LineMarkingType::LINE;
		float2 shift = 0;
		float2 scale = 1;
	};

	struct Streetlight {
		SERIALIZE(Streetlight, shift, spacing, rot, prop)

		float3 shift = 0;
		float spacing = 10;
		float rot = 0;

		AssetPtr<PropAsset> prop = dummy_asset<PropAsset>();
	};
	
	//std::string filename;

	int road_class = 0;

	float width = 16;

	float sidewalkL = -6;
	float sidewalkR = +6;

	// for now: (RHD) inner forward, outer forward, ..., inner reverse, outer reverse
	// ie. sorted from left to right per direction
	// (relevant for pathing)
	std::vector<Lane> lanes;

	std::vector<LineMarking> line_markings;
	
	std::vector<Streetlight> streetlights;
	
	AssetPtr<TrafficLightAsset> traffic_light_prop = dummy_asset<TrafficLightAsset>();
	float2 traffic_light_shift = 0;

	float speed_limit = 40 / KPH_PER_MS;


	laneid_t _num_lanes_in_dir[2];
	laneid_t num_lanes_in_dir (LaneDir dir) {
		return _num_lanes_in_dir[(int)dir];
	}

	void update_cached () {
		auto sort_less = [] (Lane const& l, Lane const& r) {
			if (l.direction != r.direction)
				return (int)l.direction < (int)r.direction;
			return l.direction == LaneDir::FORWARD ?
				l.shift < r.shift :
				l.shift > r.shift;
		};
		std::sort(lanes.begin(), lanes.end(), sort_less);
		
		_num_lanes_in_dir[(int)LaneDir::FORWARD] = 0;
		_num_lanes_in_dir[(int)LaneDir::BACKWARD] = 0;
		for (auto& lane : lanes) {
			_num_lanes_in_dir[(int)lane.direction]++;
		}
	}

	bool imgui (Options& opt) {
		bool changed = false;

		changed = ImGui::InputText("name", &name) || changed;

		changed = ImGui::DragFloat("width", &width, 0.1f) || changed;

		changed = imgui_edit_vector("lanes", lanes, [&] (int i, Lane& l) {
			bool changed = ImGui::DragFloat("shift", &l.shift, 0.1f,
				i > 0 ? lanes[i-1].shift : -INF,
				i < (int)lanes.size()-1 ? lanes[i+1].shift : +INF);
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

			//changed = l.prop.imgui() || changed;
			return changed;
		}) || changed;

		//changed = ImGui::Checkbox("has_sidewalk", &has_sidewalk) || changed;
		//if (has_sidewalk) {
			changed = ImGui::DragFloat("sidewalkL", &sidewalkL, 0.1f) || changed;
			changed = ImGui::DragFloat("sidewalkR", &sidewalkR, 0.1f) || changed;
		//}

		changed = ImGui::DragFloat2("traffic_light_shift", &traffic_light_shift.x, 0.1f) || changed;
		
		changed = opt.imgui_slider_speed("speed_limit", &speed_limit, 0, 200/KPH_PER_MS) || changed;

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

struct BuildingAsset : public Asset {
	SERIALIZE(BuildingAsset, mesh_filename, tex_filename, type, citizens, size)

	std::string mesh_filename;
	std::string tex_filename;

	BuildingType type = BuildingType::RESIDENTIAL;
	int citizens = 10;

	float3 size = 16;
	
	AssetMesh<BasicVertex> mesh;

	bool imgui (Options& opt) {
		bool changed = false;

		changed = ImGui::InputText("name", &name) || changed;
		
		changed = ImGui::Combo("type", (int*)&type, "RESIDENTIAL\0COMMERCIAL", 2) || changed;
		changed = ImGui::DragInt("citizens", &citizens, 0.1f) || changed;

		changed = ImGui::DragFloat3("size", &size.x, 0.1f) || changed;

		return changed;
	}

	void reload () {
		mesh = {}; // need to clear mesh
		assimp::load_basic(prints("assets/%s", mesh_filename.c_str()).c_str(), &mesh);
	}
};
struct VehicleAsset : public Asset {
	SERIALIZE(VehicleAsset, mesh_filename, tex_filename, spawn_weight)

	std::string mesh_filename;
	std::string tex_filename;

	float spawn_weight = 1;
	
	BoneMats bone_mats[5];

	float wheel_r = 0.5f;

	AssetMesh<SimpleAnimVertex> mesh;

	void reload () {
		mesh = {}; // need to clear mesh
		assimp::load_simple_anim(prints("assets/%s", mesh_filename.c_str()).c_str(), &mesh, bone_mats, &wheel_r);
	}
};

namespace {
	// allow from_json to query for assets by name
	// This sucks but trying to pass a Assets& context along the from_json call chain is a nightmare since the library (or my macro scheme?) is not designed for this
	// This works for singlethreaded at least (and we probably won't multithread json->objects), but even then a theadlocal global could work(?)
	Assets* g_assets = nullptr;
}

template <typename T>
struct AssetCollection {
	struct Hasher {
		using is_transparent = void; // enable heterogeneous overloads
		using is_avalanching = void; // mark class as high quality avalanching hash

		uint64_t operator() (AssetPtr<T> const& val) const noexcept {
			return this->operator()(val->name);
		}
		uint64_t operator() (std::string_view key) const noexcept {
			static_assert(std::has_unique_object_representations_v<std::string_view>);
			return std::hash<std::string_view>()(key);
		}
	};
	struct Comparer {
		using is_transparent = void;

		bool operator() (std::string_view l, AssetPtr<T> const& r) const noexcept {
			return l == r->name;
		}
		bool operator() (AssetPtr<T> const& l, AssetPtr<T> const& r) const noexcept {
			return l->name == r->name;
		}
	};

	typedef ankerl::unordered_dense::set<AssetPtr<T>, Hasher, Comparer> Hashset;

	Hashset set;

	friend SERIALIZE_TO_JSON(AssetCollection) {
		j = json::object();
		for (auto& item : t.set) {
			j.emplace(item->name, *item);
		}
	}
	friend SERIALIZE_FROM_JSON(AssetCollection) {
		for (auto& item : j.items()) {
			auto val = std::make_shared<T>();

			val->name = item.key();
			item.value().get_to(*val);

			val->reload();
			t.set.emplace(std::move(val));
		}
	}

	AssetPtr<T> operator[] (std::string_view name) {
		auto it = set.find(name);
		if (it == set.end()) {
			return dummy_asset<T>();
		}
		return *it;
	}

	auto begin () const { return set.begin(); }
	auto end () const { return set.end(); }

	void imgui (const char* label, Options& opt) {
		if (!ImGui::TreeNodeEx(label, ImGuiTreeNodeFlags_NoTreePushOnOpen)) return;
		ImGui::PushID(label);

		auto flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_Resizable;
		if (ImGui::BeginTable("##Table", 1, flags)) {
			for (auto& item : set) {
				if (ImGui::TableNextColumn()) {
					if (ImGui::TreeNode(item.get(), item->name.c_str())) {
						item->imgui(opt);
						ImGui::TreePop();
					}
				}
			}

			ImGui::EndTable();
		}
		ImGui::PopID();
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
	friend SERIALIZE_TO_JSON(Assets) { SERIALIZE_TO_JSON_EXPAND(props, traffic_lights, networks, buildings, vehicles) }
	friend SERIALIZE_FROM_JSON(Assets) {
		g_assets = &t;
		SERIALIZE_FROM_JSON_EXPAND(props, traffic_lights, networks, buildings, vehicles)
		t.assets_reloaded = true;
	}

	bool assets_reloaded = false;

	AssetCollection<PropAsset>         props;
	AssetCollection<TrafficLightAsset> traffic_lights;
	AssetCollection<NetworkAsset>      networks;
	AssetCollection<BuildingAsset>     buildings;
	AssetCollection<VehicleAsset>      vehicles;
	
	template <typename T>
	AssetPtr<T> query (std::string_view str);

	template<> AssetPtr<PropAsset> query<PropAsset> (std::string_view str) {
		return props[str];
	}
	template<> AssetPtr<TrafficLightAsset> query<TrafficLightAsset> (std::string_view str) {
		return traffic_lights[str];
	}

	void reload_all () {
		for (auto& a : props         ) a->reload();
		for (auto& a : traffic_lights) a->reload();
		for (auto& a : networks      ) a->reload();
		for (auto& a : buildings     ) a->reload();
		for (auto& a : vehicles      ) a->reload();
		assets_reloaded = true;
	}

	void imgui (Options& opt) {
		if (!imgui_Header("Assets")) return;

		if (ImGui::Button("Reload All"))
			reload_all();
		
		props         .imgui("props",          opt);
		traffic_lights.imgui("traffic_lights", opt);
		networks      .imgui("networks",       opt);
		buildings     .imgui("buildings",      opt);

		ImGui::PopID();
	}

};

template <typename T> void to_json (json& j, AssetPtr<T> const& ptr) {
	j = ptr->name;
}
template <typename T> void from_json<> (json const& j, AssetPtr<T>& ptr) {
	ptr = g_assets->query<T>(j.get<std::string>());
}

