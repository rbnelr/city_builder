#pragma once
#include "common.hpp"
#include "bezier.hpp"

// TODO: Is there a resonable way of allowing whole numbers (30, 45, 70 etc.) of speed in both unit systems to match when switching?
inline constexpr float KPH_PER_MS = 3.6f;
inline constexpr float MPH_PER_MS = 2.23693632f;

enum class SpeedUnit {
	MS,
	KPH,
	MPH,
};
inline constexpr float SpeedUnitPerMs[] = {
	1, KPH_PER_MS, MPH_PER_MS
};
inline constexpr const char* SpeedUnitStr[] = {
	"m/s", "km/h", "mph"
};
NLOHMANN_JSON_SERIALIZE_ENUM(SpeedUnit, { {SpeedUnit::MS,"MS"}, {SpeedUnit::KPH,"KPH"}, {SpeedUnit::MPH,"MPH"} })

// "Global" options
struct Options {
	SERIALIZE(Options, speed_unit)

	SpeedUnit speed_unit = SpeedUnit::KPH;
	
	static std::string format_speed (float speed, SpeedUnit unit) {
		return prints("%.0f %s", speed * SpeedUnitPerMs[(int)unit], SpeedUnitStr[(int)unit]);
	}
	std::string format_speed (float speed) {
		return format_speed(speed, speed_unit);
	}

	bool imgui_slider_speed (const char* label, float* speed, float min, float max) {
		float fac = SpeedUnitPerMs[(int)speed_unit];

		float val = *speed * fac;
		min *= fac;
		max *= fac;

		bool ret = ImGui::SliderFloat(
			prints("%s (%s)",label, SpeedUnitStr[(int)speed_unit]).c_str(),
			&val, min, max);

		*speed = val / fac;
		return ret;
	}

	void imgui () {
		if (!imgui_Header("Options")) return;
		
		ImGui::Combo("speed_unit", (int*)&speed_unit, SpeedUnitStr, ARRLEN(SpeedUnitStr));

		ImGui::PopID();
	}
};

#ifdef _WIN32
#include "engine/kisslib/clean_windows_h.hpp"
#include <psapi.h>

inline void imgui_process_stats (bool* open) {
	if (!ImGui::Begin("Process Stats", open)) return;

	auto hproc = GetCurrentProcess();

	PROCESS_MEMORY_COUNTERS pmc = {};
	if (GetProcessMemoryInfo(hproc, &pmc, sizeof(pmc))) {
#define FMT(x) ((float)x / (1024.f*1024.f))

		ImGui::Text( "PageFaultCount:             %08llu",        pmc.PageFaultCount              );
		ImGui::Text( "PeakWorkingSetSize:         %08.3f MB", FMT(pmc.PeakWorkingSetSize        ) );
		ImGui::Text( "WorkingSetSize:             %08.3f MB", FMT(pmc.WorkingSetSize            ) );
		ImGui::Text( "QuotaPeakPagedPoolUsage:    %08.3f MB", FMT(pmc.QuotaPeakPagedPoolUsage   ) );
		ImGui::Text( "QuotaPagedPoolUsage:        %08.3f MB", FMT(pmc.QuotaPagedPoolUsage       ) );
		ImGui::Text( "QuotaPeakNonPagedPoolUsage: %08.3f MB", FMT(pmc.QuotaPeakNonPagedPoolUsage) );
		ImGui::Text( "QuotaNonPagedPoolUsage:     %08.3f MB", FMT(pmc.QuotaNonPagedPoolUsage    ) );
		ImGui::Text( "PagefileUsage:              %08.3f MB", FMT(pmc.PagefileUsage             ) ); 
		ImGui::Text( "PeakPagefileUsage:          %08.3f MB", FMT(pmc.PeakPagefileUsage         ) );

		// TODO: Cpu use?
	}

	ImGui::End();
}
#endif


inline float angle2d (float2 dir) {
	return length_sqr(dir) > 0 ? atan2f(dir.y, dir.x) : 0;
}

// Argably, rotating like this is a mistake, since I use it to get right from forward vectors
// which either should be with z=0 or needs to be rethought!
//inline float3 rotateZ_90 (float3 v) {
//	return float3(-v.y, v.x, v.z);
//}
//inline float3 rotateZ_CW90 (float3 v) {
//	return float3(v.y, -v.x, v.z);
//}

struct Dirs {
	float3 forw, right, up;
};
inline Dirs relative2dir (float3 forward) {
	Dirs d;
	d.forw = forward;
	// right vector always level, unless some kind of tilt is wanted?
	d.right = float3(forward.y, -forward.x, 0);
	d.up = cross(d.right, d.forw);
	return d;
}

inline bool line_line_intersect (float2 const& a, float2 const& b, float2 const& c, float2 const& d, float2* out_point) {
	// https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
	float numer = (a.x-c.x)*(c.y-d.y) - (a.y-c.y)*(c.x-d.x);
	float denom = (a.x-b.x)*(c.y-d.y) - (a.y-b.y)*(c.x-d.x);
	if (denom == 0)
		return false; // parallel, either overlapping (numer == 0) or not
	float u = numer / denom;
	*out_point = a + u*(b-a);
	return true; // always intersect for now
}

inline _FORCEINLINE bool line_line_seg_intersect (float2 const& a, float2 const& ab, float2 const& c, float2 const& cd, float* out_u, float* out_v) {
	float2 ac = c - a;

	float denom = ab.x*cd.y - ab.y*cd.x;
	if (denom == 0)
		return false; // parallel, either overlapping (numer == 0) or not
	
	float numer_ab = ac.x*cd.y - ac.y*cd.x;
	float numer_cd = ac.x*ab.y - ac.y*ab.x;
	float u = numer_ab / denom;
	float v = numer_cd / denom;
	if (u < 0.0f || u > 1.0f || v < 0.0f || v > 1.0f)
		return false;

	*out_u = u;
	*out_v = v;
	return true;
}

inline bool intersect_circle_ray (float3 pos, float r, Ray const& ray, float* hit_dist) {
	float t = 0;
	if (ray.dir.z == 0) {
		if (ray.pos.z != pos.z) return false; // miss
	}
	else {
		t = (pos.z - ray.pos.z) * (1.0f / ray.dir.z);
	}

	float x = ray.pos.x + ray.dir.x * t - pos.x;
	float y = ray.pos.y + ray.dir.y * t - pos.y;

	float dist_sqr = x*x + y*y;
	if (dist_sqr > r*r) return false;
	
	*hit_dist = sqrt(dist_sqr);
	return true;
}

#if 0
inline float line_line_dist_sqr (float2 a, float2 b, float2 c, float2 d, float* out_u, float* out_v) {
	constexpr float eps = 0.0001f;
	
	float2 ab = b - a;
	float2 cd = d - c;

	float2 ac = c - a;

	float denom = ab.x*cd.y - ab.y*cd.x;
	if (abs(denom) > eps) {
		//denom = 1.0f / denom;
		float u = (ac.x*cd.y - ac.y*cd.x) / denom;
		float v = (ac.x*ab.y - ac.y*ab.x) / denom;

		if (u >= 0.0f && v >= 0.0f && u <= 1.0f && v <= 1.0f) {
			*out_u = u;
			*out_v = v;
			return 0.0f; // intersecting
		}
	}

	// TODO: early out in parallel case (denom == 0) ?
	
	float2 ca = a - c;
	float2 cb = b - c;
	float2 ad = d - a;

	float u0=0.0f, u1=0.0f, v0=0.0f, v1=0.0f;

	float cd_len = length_sqr(cd); // TODO: should not happen in our inputs
	if (cd_len != 0.0f) {
		v0 = clamp(dot(cd, ca) / cd_len, 0.0f, 1.0f);
		v1 = clamp(dot(cd, cb) / cd_len, 0.0f, 1.0f);
	}
	float ab_len = length_sqr(ab);
	if (ab_len != 0.0f) {
		u0 = clamp(dot(ab, ac) / ab_len, 0.0f, 1.0f);
		u1 = clamp(dot(ab, ad) / ab_len, 0.0f, 1.0f);
	}
	
	float len0 = length_sqr((v0 * cd) - ca); // TODO: ac - (v0 * cd) -> avoid ca compute
	float len1 = length_sqr((v1 * cd) - cb);
	float len2 = length_sqr((u0 * ab) - ac);
	float len3 = length_sqr((u1 * ab) - ad);
	
	float min_len;
	float u, v;
	{
		min_len = len0;
		u = 0.0f; v = v0;
	}
	if (len1 < min_len) {
		min_len = len1;
		u = 1.0f; v = v1;
	}
	if (len2 < min_len) {
		min_len = len2;
		u = u0; v = 0.0f;
	}
	if (len3 < min_len) {
		min_len = len3;
		u = u1; v = 1.0f;
	}
	
	*out_u = u;
	*out_v = v;
	return min_len;
}

inline _FORCEINLINE bool ray_box_intersection (float2 a, float2 da, float2 b, float2 db, float len_b, float r, float* out_u) {
	db /= len_b;
	float2 db_n = float2(-db.y, db.x);

	float x = dot(a - b, db);
	float dx = dot(da, db);
	
	float y = dot(a - b, db_n);
	float dy = dot(da, db_n);

	float offs_x0 =        -r  - x;
	float offs_x1 = (len_b +r) - x;
	float offs_y0 = -r - y;
	float offs_y1 = +r - y;

	float tx0 = -INF, tx1 = +INF;
	if (dx != 0.0f) {
		dx = 1.0f / dx;
		float t0 = offs_x0 * dx;
		float t1 = offs_x1 * dx;
		tx0 = min(t0, t1);
		tx1 = max(t0, t1);
	}
	else {
		if (offs_x0 > 0.0f || offs_x1 < 0.0f)
			return false; // miss
	}
	
	float ty0 = -INF, ty1 = +INF;
	if (dy != 0.0f) {
		dy = 1.0f / dy;
		float t0 = offs_y0 * dy;
		float t1 = offs_y1 * dy;
		ty0 = min(t0, t1);
		ty1 = max(t0, t1);
	}
	else {
		if (offs_y0 > 0.0f || offs_y1 < 0.0f)
			return false; // miss
	}

	float t0 = max(tx0, ty0);
	float t1 = min(tx1, ty1);

	t0 = max(t0, 0.0f);

	if (t0 > t1 || t0 > 1.0f)
		return false;

	*out_u = t0;
	return true;
}
#endif

inline bool intersect_ray_zplane (Ray const& ray, float plane_z, float* hit_t) {
	if (ray.dir.z == 0.0f)
		return false;

	float t = (plane_z - ray.pos.z) / ray.dir.z;
	if (t < 0.0f)
		return false;

	*hit_t = t;
	return true;
}
inline bool intersect_ray_zcircle (Ray const& ray, float3 center, float r, float* hit_t=nullptr, float2* hit_point=nullptr) {
	float t;
	if (!intersect_ray_zplane(ray, center.z, &t))
		return false;

	float2 xy = (float2)ray.pos + (float2)ray.dir * t;
	float dist_sqr = length_sqr(xy - (float2)center);
	if (dist_sqr > r*r)
		return false;

	if (hit_t) *hit_t = t;
	if (hit_point) *hit_point = xy;
	return true;
}

template <typename... TYPES>
class NullableVariant {
	typedef std::monostate null_t;
	std::variant<null_t, TYPES...> var;
public:

	template <typename T>
	NullableVariant (T ref) {
		assert((bool)ref); // Assert that normal references are never null (should have assigned nullptr instead)
		var = ref;
	}

	template<>
	NullableVariant (nullptr_t) {
		var = null_t();
	}

	NullableVariant () {
		var = null_t();
	}

	template<typename... Ts> struct overloaded : Ts... { using Ts::operator()...; };
	template<typename... Ts> overloaded(Ts...) -> overloaded<Ts...>;

	template <typename FUNC>
	auto visit (FUNC func) {
		return std::visit(overloaded{
			[] (null_t x) { assert(false); _UNREACHABLE; },
			func,
		}, var);
	}
	
	template <typename T>
	T get () {
		return std::holds_alternative<T>(var) ? std::get<T>(var) : nullptr;
	}

	explicit operator bool () const { return var.index() != 0; }
	bool operator== (NullableVariant const& r) const { return var == r.var; }
	bool operator!= (NullableVariant const& r) const { return var != r.var; }
};

struct SelCircle {
	float3   pos;
	float    radius;
	lrgb     col;

	bool test (Ray const& ray, float* hit_dist) {
		return intersect_circle_ray(pos, radius, ray, hit_dist);
	}
	
	void highlight () {
		g_dbgdraw.wire_circle(pos, radius + 0.25f, lrgba(1,1,1,1), 32);
	}
	void highlight_selected (float tint=0, lrgba tint_col=0) {
		lrgba tinted = lerp(lrgba(col,1), tint_col, tint);
		g_dbgdraw.wire_circle(pos, radius, tinted, 32);
	}
};

inline void draggable (Input& I, View3D& view, float3* pos, float r) {
	// use ptr to identiy if and what we are dragging, to allow using this function with any number of items
	static float3* dragging_ptr = nullptr;
	static float2 drag_offs;
	
	bool hit = false;
	float2 hit_point;

	Ray ray;
	if (view.cursor_ray(I, &ray.pos, &ray.dir)) {
		hit = intersect_ray_zcircle(ray, float3(*pos, 0), r, nullptr, &hit_point);
	}
	if (hit && (dragging_ptr == nullptr || dragging_ptr == pos)) {
		g_dbgdraw.wire_circle(float3(*pos, 0), r, lrgba(1,1,0,1));
	}

	if (!dragging_ptr && hit && I.buttons[MOUSE_BUTTON_LEFT].went_down) {
		drag_offs = hit_point - (float2)*pos;
		dragging_ptr = pos;
	}
	if (dragging_ptr == pos) {
		if (hit && I.buttons[MOUSE_BUTTON_LEFT].is_down) {
			*pos = float3(hit_point - drag_offs, pos->z);
		}
		else {
			dragging_ptr = nullptr;
		}
	}
}

template <typename KEY_T, typename VAL_T, typename HASHER=std::hash<KEY_T>, typename EQUAL=std::equal_to<KEY_T>>
struct Hashmap : public std::unordered_map<KEY_T, VAL_T, HASHER> {
	// Not tested with move only types yet (avoiding copies can be hard with try_add etc)

	template <typename T, typename V>
	VAL_T* try_add (T&& key, V&& val) {
		auto ret = this->try_emplace(key, val);
		return ret.second ? &ret.first->second : nullptr; // lmao, nice API guys
	}

	template <typename T, typename V>
	VAL_T& add (T&& key, V&& val) {
		auto* ptr = try_add(key, val);
		assert(ptr);
		return *ptr;
	}

	template <typename T>
	VAL_T* try_get (T const& key) {
		auto it = this->find(key);
		return it != this->end() ? &it->second : nullptr;
	}

	template <typename T, typename FUNC>
	VAL_T& get_or_create (T const& key, FUNC create) {
		auto it = this->find(key);
		if (it != this->end())
			return it->second;

		// have to hash twice since afaik there is no API for this
		// if unnecessary create is ok  this->emplace(key, create())  works
		// if create() is default ctor  return *this[key];  works
		auto res = this->emplace(key, create());
		return res.first->second;
	}

	template <typename T>
	VAL_T& operator[] (T const& key) {
		auto* ptr = try_get(key);
		assert(ptr);
		// deref null, hopefully crash instead of causing bugs like unordered_map::operator[] does by inserting a bogous value
		return *ptr;
	}
};

template <typename T, typename GET_KEY, typename GET_IDX>
struct MinHeapFunc {
	std::vector<T>& items;
	
	GET_KEY get_key;
	GET_IDX get_idx;

	static int first_child_index (int idx) {
		return (idx * 2) + 1;
	}
	static int parent_index (int idx) {
		return (idx - 1) / 2;
	}

	static void verify_heap (T* items, int count) {
		for (int i=0; i<(int)items.size(); ++i) {
			int first_child = first_child_index(i);

			// equality breaks parent < child assert, check !child < parent instead
			for (int i=0; i<2; ++i) {
				if (first_child + i < count) {
					assert(!( get_key(items[first_child + i]) > get_key(items[i]) ));
				}
			}

			assert(get_idx(items[i]) == i);
		}
	}
	static void verify_heap2 (T* items, int count) {
		for (int i=0; i<(int)items.size(); ++i) {
			assert(get_idx(items[i]) == i);
		}
	}

	void swap_with_indices (T* items, int a, int b) {
		T val_a = items[a];
		T val_b = items[b];

		items[a] = val_b;
		items[b] = val_a;

		get_idx(val_a) = a;
		get_idx(val_b) = b;
	}

	void bubble_up (T* items, int count, int idx) {
		assert(idx >= 0 && idx < count);
		//verify_heap2();

		for (;;) {
			int parent = parent_index(idx);
			if (parent < 0)
				break;

			// if child < parent swap to keep min heap property
			// note: avoid child <= parent for slight speed up and to avoid fifo behavior in edge cases (prio queue)
			auto cur_key    = get_key(items[idx]);
			auto parent_key = get_key(items[parent]);
			if (!(cur_key < parent_key))
				break;
			
			swap_with_indices(items, idx, parent);

			idx = parent;

			//verify_heap2();
		}

		//verify_heap();
	}
	void bubble_down (T* items, int count, int idx) {
		assert(idx >= 0 && idx < count);
		//verify_heap2();

		for (;;) {
			int left_child  = first_child_index(idx);
			int right_child = left_child+1;

			if (left_child >= count)
				break; // no children -> idx is leaf
			
			int least = left_child;
			auto least_key = get_key(items[left_child]);

			// find least child if right exists else take left
			if (right_child < count) {
				auto right_key = get_key(items[right_child]);

				bool cmp = right_key < least_key;
				least     = cmp ? right_child : least;
				least_key = cmp ? right_key : least_key;
			}

			// if child < parent swap to keep min heap property
			auto cur_key = get_key(items[idx]);
			if (!(least_key < cur_key))
				break;
			
			swap_with_indices(items, least, idx);

			idx = least;

			//verify_heap2();
		}

		//verify_heap();
	}

	_FORCEINLINE void push (T item) {
		//verify_heap();

		int new_i = (int)items.size();
		items.push_back(std::move(item));

		get_idx(items[new_i]) = new_i;
		
		bubble_up(items.data(), (int)items.size(), new_i);
	}
	_FORCEINLINE T pop () {
		//verify_heap();

		assert(items.size() > 0);
		// get first item
		T res = std::move(items[0]);

		bool has_remain = items.size() > 1;
		
		// swap in last item
		if (has_remain) {
			items[0] = std::move(items[items.size()-1]);
			get_idx(items[0]) = 0;
		}
		get_idx(res) = -1;

		// shrink vec
		items.pop_back();

		if (has_remain) {
			bubble_down(items.data(), (int)items.size(), 0);
		}
		return res;
	}
	
	_FORCEINLINE void decreased_at (int idx) {
		bubble_up(items.data(), (int)items.size(), idx);
	}

	// push new item or decrease key of existing item
	// new item is pushed if key in item is -1
	_FORCEINLINE void push_or_decrease (T item) {
		//verify_heap();

		int idx = get_idx(item);
		if (idx < 0) {
			// push item with no cached index

			int new_i = (int)items.size();
			items.push_back(std::move(item));

			get_idx(items[new_i]) = new_i;
		
			idx = new_i;
		}

		bubble_up(items.data(), (int)items.size(), idx);
	}
};
