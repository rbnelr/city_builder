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

struct OverlayDraw {
	
	//template <typename FUNC>
	//void push_polygon (lrgba col, FUNC add_point) {
	//
	//	while ()
	//}
};
