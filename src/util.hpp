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

size_t imgui_process_stats ();

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

template<class... Ts>
struct overloaded : Ts... { using Ts::operator()...; };

template<class U, class... Ts>
auto visit_overloaded (U& var, Ts... ts) {
	return std::visit(overloaded{ts...}, var);
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
		g_dbgdraw.wire_circle(pos, radius + 0.25f, lrgba(col,1), 32);
	}
	void highlight_selected (float tint=0, lrgba tint_col=0) {
		lrgba tinted = lerp(lrgba(col,1), tint_col, tint);
		g_dbgdraw.wire_circle(pos, radius, tinted, 32);
	}
};
struct SelRect {
	float3   pos;
	float3   forw;
	float3   right;
	lrgb     col;

	bool test (Ray const& ray, float* hit_dist) {
		return intersect_rect_ray(pos, forw, right, ray, hit_dist);
	}
	
	void highlight () {
		float3 f = normalizesafe(forw) * 0.25f;
		float3 r = normalizesafe(right) * 0.25f;

		float3 p = pos;
		p -= f * 0.5f;
		p -= r * 0.5f;
		g_dbgdraw.wire_quad(p, forw + f, right + r, lrgba(col,1));
	}
	void highlight_selected (float tint=0, lrgba tint_col=0) {
		lrgba tinted = lerp(lrgba(col,1), tint_col, tint);
		g_dbgdraw.wire_quad(pos, forw, right, tinted);
	}
};

inline bool draggable (Input& I, View3D& view, float3* pos, float r) {
	// use ptr to identiy if and what we are dragging, to allow using this function with any number of items
	static float3* dragging_ptr = nullptr;
	static float2 grab_offs;
	
	bool valid = false;
	bool hovered = false;
	float2 cursor_pos;

	Ray ray;
	if (view.cursor_ray(I, &ray.pos, &ray.dir)) {
		int res = intersect_ray_zcircle(ray, float3(*pos, 0), r, nullptr, &cursor_pos);
		valid = res >= 0;
		hovered = res > 0;
	}

	bool hovered_or_dragging = false;
	// highlight circle unless dragging something else already
	if (hovered && (dragging_ptr == nullptr || dragging_ptr == pos)) {
		g_dbgdraw.wire_circle(float3(*pos, 0), r, lrgba(1,1,0,1));
		hovered_or_dragging = true;
	}

	// start dragging with LMB on circle
	if (hovered && dragging_ptr == nullptr && I.buttons[MOUSE_BUTTON_LEFT].went_down) {
		grab_offs = cursor_pos - (float2)*pos;
		dragging_ptr = pos;
	}
	if (dragging_ptr == pos) {
		// while dragging, move object with cursor
		if (I.buttons[MOUSE_BUTTON_LEFT].is_down) {
			if (valid) {
				*pos = float3(cursor_pos - grab_offs, pos->z);
				// snapping feature for convenience
				if (I.buttons[KEY_LEFT_CONTROL].is_down)
					*pos = round(*pos);
			}
		}
		// stop dragging when button up
		else {
			dragging_ptr = nullptr;
		}
	}

	return hovered_or_dragging;
}

inline bool wait_for (float& timer, float dt) {
	timer -= dt;
	if (timer <= 0.0f) {
		timer = 0;
		return true;
	}
	return false;
}

template <typename KEY_T, typename VAL_T, typename HASHER=std::hash<KEY_T>, typename EQUAL=std::equal_to<KEY_T>>
struct Hashmap : public std::unordered_map<KEY_T, VAL_T, HASHER> {
	// Not tested with move only types yet (avoiding copies can be hard with try_add etc)

	size_t approx_alloc_size () const {
		typedef std::unordered_map<KEY_T, VAL_T, HASHER> Base;

		// https://stackoverflow.com/questions/25375202/how-to-measure-the-memory-usage-of-stdunordered-map
		return (size_t)(
			(Base::size() * (sizeof(VAL_T) + sizeof(void*)) + // data list
			Base::bucket_count() * (sizeof(void*) + sizeof(size_t))) // bucket index
			* 1.5); // estimated allocation overheads
	}

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
	VAL_T& get_or_default (T const& key) {
		return std::unordered_map<KEY_T, VAL_T, HASHER>::operator[](key);
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

struct CurvedDecalVertex {
	float3 pos0, pos1;
	float3 right0, right1; // right vector for decal boxes
	float  height; // height for decal boxes
	float  uv0, uv1;
	float  uv_len; // needed for pattern rendering
	int    tex;
	float4 tint0, tint1;

	VERTEX_CONFIG(
		ATTRIB(FLT,3, CurvedDecalVertex, pos0),
		ATTRIB(FLT,3, CurvedDecalVertex, pos1),
		ATTRIB(FLT,3, CurvedDecalVertex, right0),
		ATTRIB(FLT,3, CurvedDecalVertex, right1),
		ATTRIB(FLT,1, CurvedDecalVertex, height),
		ATTRIB(FLT,1, CurvedDecalVertex, uv0),
		ATTRIB(FLT,1, CurvedDecalVertex, uv1),
		ATTRIB(FLT,1, CurvedDecalVertex, uv_len),
		ATTRIB(INT,1, CurvedDecalVertex, tex),
		ATTRIB(FLT,4, CurvedDecalVertex, tint0),
		ATTRIB(FLT,4, CurvedDecalVertex, tint1),
	)
};
struct CurvedDecals {
	typedef uint32_t IDX_T;

	std::vector<CurvedDecalVertex> vertices;
	//std::vector<IDX_T>             indices;
	
	void clear () {
		vertices.clear();
		//indices.clear();
		vertices.shrink_to_fit();
		//indices.shrink_to_fit();
	}
	void reserve (size_t num) {
		vertices.reserve(num);
	}

	void _push_bezier (Bezier3 const& bez, float2 size, int tex,
			lrgba col0, lrgba col1, float2 t_range, float2 uv_range, int res) {

		//size_t idx = vertices.size();
		auto* verts = push_back(vertices, res);
		//auto* indxs = push_back(indices , res*2);

		bool first = true;
		float3 prev_pos   = 0;
		float3 prev_right = 0;
		float  prev_uv    = 0;
		float4 prev_tint  = 0;

		for (int i=0; i <= res; ++i) {
			float curve_t = (float)i * (1.0f / (float)res);
			auto val = bez.eval(lerp(t_range.x, t_range.y, curve_t));

			float3 forw = normalizesafe(val.vel);
			float3 right = rotate90_right(forw) * (size.x*0.5f);
			
			CurvedDecalVertex v;
			v.pos0 = prev_pos;
			v.pos1 = val.pos;
			v.right0 = prev_right;
			v.right1 = right;
			v.height = size.y;
			v.uv0 = prev_uv;
			v.uv1 = lerp(uv_range.x, uv_range.y, curve_t); // scales texture
			v.uv_len = uv_range.y - uv_range.x;
			v.tex = tex;
			v.tint0 = prev_tint;
			v.tint1 = lerp(col0, col1, curve_t); // lerp color for lane wear weighting

			if (!first) {
				*verts++ = v;
			}
			first = false;
			
			prev_pos   = v.pos1;
			prev_right = v.right1;
			prev_uv    = v.uv1;
			prev_tint  = v.tint1;
		}

		//for (int i=0; i<res; ++i) {
		//	*indxs++ = (IDX_T)idx++;
		//	*indxs++ = (IDX_T)idx;
		//
		//	assert(std::in_range<IDX_T>(idx));
		//}
	}

	void push_arrow (Bezier3 const& bez, float2 size, int tex, lrgba col, float2 t_range=float2(0,1), int res=16) {
		float len = bez.approx_len(res) * (t_range.y - t_range.x);
		float uv_aspect = 2.0f * size.x / len; // 1:2 aspect ratio texture
		
		// keep tip and tail of arrow correct aspect ratio, and stretch the straight part
		const float tail_uv = 0.1f;
		const float tip_uv  = 0.3f;

		const int tips_n = 2;
		const int middle_n = 14;
		const int total_n = middle_n + tips_n*2;

		// uv spots of tail/tip
		float uv0 = tail_uv;
		float uv1 = 1.0f - tip_uv;

		// corrected to keep uvs aspect ratio constant
		float t0 = tail_uv * uv_aspect;
		float t1 = 1.0f - tip_uv * uv_aspect;

		if (t1 - t0 < 0.01f) {
			// straight section is 0 length, need to shrink tail/tip!
			t0 = t1 = tail_uv / tip_uv;
		}
		

		//size_t idx = vertices.size();
		auto* verts = push_back(vertices, total_n);
		//auto* indxs = push_back(indices , total_n*2);
		
		//int prev_idx = -1;

		bool first = true;
		float3 prev_pos   = 0;
		float3 prev_right = 0;
		float  prev_uv    = 0;

		auto push = [&] (float t0, float t1, float uv0, float uv1, int count, bool last=false) {
			// last vertex of cur section is first of next section unless last section
			for (int i=0; i < (last ? count+1 : count); ++i) {
				float section_t = (float)i * (1.0f / (float)count);
				float adjusted_t = lerp(t0, t1, section_t);

				auto val = bez.eval(lerp(t_range.x, t_range.y, adjusted_t));

				float3 forw = normalizesafe(val.vel);
				float3 right = rotate90_right(forw) * (size.x*0.5f);
			
				CurvedDecalVertex v;
				v.pos0 = prev_pos;
				v.pos1 = val.pos;
				v.right0 = prev_right;
				v.right1 = right;
				v.height = size.y;
				v.uv0 = prev_uv;
				v.uv1 = lerp(uv0, uv1, section_t);
				v.uv_len = 1;
				v.tex = tex;
				v.tint0 = col;
				v.tint1 = col;
				
				if (!first) {
					*verts++ = v;
				}
				first = false;
			
				prev_pos   = v.pos1;
				prev_right = v.right1;
				prev_uv    = v.uv1;
			}
		};

		push( 0, t0,    0, uv0,  tips_n);
		push(t0, t1,  uv0, uv1,  middle_n); // degenerate in the shrink case!
		push(t1,  1,  uv1,   1,  tips_n, true);
		
		//for (int i=0; i<total_n; ++i) {
		//	*indxs++ = (IDX_T)idx++;
		//	*indxs++ = (IDX_T)idx;
		//	
		//	assert(std::in_range<IDX_T>(idx));
		//}
	}

	void push_bezier_color_lerp (Bezier3 const& bez, float2 size, int tex,
			lrgba col0, lrgba col1, float2 t_range=float2(0,1), int res=12) {
		float len = bez.approx_len(res) * (t_range.y - t_range.x);
		float uv_len = len / size.x; // scale uv aspect to fit length (shader will adjust for texture aspect)
		
		_push_bezier(bez, size, tex, col0, col1, t_range, float2(0, uv_len), res);
	}
	void push_bezier (Bezier3 const& bez, float2 size, int tex,
			lrgba col, float2 t_range=float2(0,1), int res=12) {
		push_bezier_color_lerp(bez, size, tex, col, col, t_range, res);
	}
};

struct OverlayDraw {
	CurvedDecals curves;

	enum BezierOverlayPattern : int {
		PATTERN_SOLID=0,
		PATTERN_STRIPED,
		TEXTURE_THICK_ARROW,
		TEXTURE_THIN_ARROW,
	};

	void begin () {
		curves.clear();
	}
};

class MemUse {
	typedef const char* Name;

	struct Entry {
		size_t count = 0;
		size_t size = 0;
	};
	std::unordered_map<Name, Entry> sizes;
	
public:
	void begin () {
		sizes.clear();
	}
	void add (Name name, size_t size) {
		auto& entry = sizes[name];
		entry.count++;
		entry.size += size;
	}
	
	void _imgui ();

	template <typename FUNC>
	void imgui (FUNC add_all) {
		if (!imgui_Header("Mem Use")) return;

		add_all();

		_imgui();
		ImGui::PopID();
	}

	template <typename T>
	static size_t sizeof_alloc (std::vector<T> const& vec) {
		return vec.capacity() * sizeof(T);
	}

	template <typename T, typename U, typename V>
	static size_t sizeof_alloc (Hashmap<T, U, V> const& hm) {
		return hm.approx_alloc_size();
	}
};
