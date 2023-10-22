#pragma once
#include "common.hpp"

inline constexpr float KPH_PER_MS = 3.6f;
inline constexpr float MPH_PER_MS = 2.23693632f;

enum SpeedUnit : int {
	UNIT_MS,
	UNIT_KPH,
	UNIT_MPH,
};
inline constexpr float SpeedUnitPerMs[] = {
	1, KPH_PER_MS, MPH_PER_MS
};
inline constexpr const char* SpeedUnitStr[] = {
	"m/s", "km/h", "mph"
};

inline std::string format_speed (float speed, SpeedUnit unit) {
	return prints("%.0f %s", speed * SpeedUnitPerMs[unit], SpeedUnitStr[unit]);
}

struct Settings {
	SpeedUnit speed_unit = UNIT_KPH;

	void imgui () {
		if (!ImGui::TreeNode("Settings")) return;
		
		ImGui::Combo("speed_unit", (int*)&speed_unit, SpeedUnitStr, ARRLEN(SpeedUnitStr));

		ImGui::TreePop();
	}
};

inline bool imgui_slider_speed (Settings& settings, const char* label, float* speed, float min, float max) {
	float fac = SpeedUnitPerMs[settings.speed_unit];

	float val = *speed * fac;
	min *= fac;
	max *= fac;

	bool ret = ImGui::SliderFloat(
		prints("%s (%s)",label, SpeedUnitStr[settings.speed_unit]).c_str(),
		speed, min, max);

	*speed = val / fac;
	return ret;
}



inline float angle2d (float2 dir) {
	return length_sqr(dir) > 0 ? atan2f(dir.y, dir.x) : 0;
}

struct BezierRes {
	float2 pos;
	float2 vel;   // velocity (over bezier t)
	float  curv;  // curvature (delta angle over dist along curve)
};
struct Bezier3 {
	float2 a;
	float2 b;
	float2 c;

	BezierRes eval (float t) const {
		float2 c0 = a;           // a
		float2 c1 = 2 * (b - a); // (-2a +2b)t
		float2 c2 = a - 2*b + c; // (a -2b +c)t^2
		
		float t2 = t*t;

		float2 value = c2*t2    + c1*t + c0; // f(t)
		float2 deriv = c2*(t*2) + c1;        // f'(t)

		return { value, deriv };
	}

	BezierRes eval_with_curv (float t) const {
		//float2 ab = lerp(a, b, t);
		//float2 bc = lerp(b, c, t);
		//return lerp(ab, bc, t);
		
		//float t2 = t*t;
		//
		//float _2t1 = 2.0f*t;
		//float _2t2 = 2.0f*t2;
		//
		//float ca = 1.0f -_2t1   +t2;
		//float cb =       _2t1 -_2t2;
		//float cc =               t2;
		//
		//return ca*a + cb*b + cc*c;

		float2 c0 = a;           // a
		float2 c1 = 2 * (b - a); // (-2a +2b)t
		float2 c2 = a - 2*b + c; // (a -2b +c)t^2
		
		float t2 = t*t;

		float2 value = c2*t2    + c1*t + c0; // f(t)
		float2 deriv = c2*(t*2) + c1;        // f'(t)
		float2 accel = c2*2;                 // f''(t)

		
		// angle of movement can be gotten via:
		// ang = atan2(deriv.y, deriv.x)

		// curvature can be defined as change in angle
		// atan2(deriv.y, deriv.x)
		// atan2 just offsets the result based on input signs, so derivative of atan2
		// should be atan(y/x)
		
		// wolfram alpha: derive atan(y(t)/x(t)) with respect to x:
		// gives me (x*dy - dx*y) / (x^2+y^2) (x would be deriv.x and dy would be accel.x)
		// this formula from the https://math.stackexchange.com/questions/3276910/cubic-b%c3%a9zier-radius-of-curvature-calculation?rq=1
		// seems to divide by the length of the sqrt(denom) as well, normalizing it by length(vel)
		// vel = dpos / t -> (x/dt) / (dpos/dt) -> x/dpos
		// so it seems this actually normalizes the curvature to be decoupled from your t (parameter) space
		// and always be correct in position space
		float denom = deriv.x*deriv.x + deriv.y*deriv.y;
		float curv = (deriv.x*accel.y - accel.x*deriv.y) / (denom * sqrt(denom)); // denom^(3/2)

		return { value, deriv, curv };
	}

	// Optimization if compier is not smart enough to optimize loop invariant
	struct Coefficients {
		float2 c0;
		float2 c1;
		float2 c2;
	};
	Coefficients get_coeff () {
		Coefficients co; 
		co.c0 = a;           // a
		co.c1 = 2 * (b - a); // (-2a +2b)t
		co.c2 = a - 2*b + c; // (a -2b +c)t^2
		return co;
	}
	float2 eval_value (Coefficients const& co, float t) {
		float t2 = t*t;
		return co.c2*t2 + co.c1*t + co.c0;
	}
	
	// it's faster for check_conflict because compiler is dum dum
	float2 eval_value_fast_t (float t) const {
		float t2 = t*t;
		
		float _2t1 = 2.0f*t;
		float _2t2 = 2.0f*t2;
		
		float ca = 1.0f -_2t1   +t2;
		float cb =       _2t1 -_2t2;
		float cc =               t2;
		
		float2 v;
		v.x = ca*a.x + cb*b.x + cc*c.x;
		v.y = ca*a.y + cb*b.y + cc*c.y;
		return v;
	}

	float approx_len (int steps) {
		auto co = get_coeff();
		float2 prev = a;

		float len = 0;
		for (int i=0; i<steps; ++i) {
			float t = (float)(i+1) * (1.0f / steps);
			float2 pos = eval_value(co, t);
			len += length(pos - prev);
			prev = pos;
		}

		return len;
	}

	void dbg_draw (View3D const& view, float z, int res, lrgba col, float t0=0, float t1=1) const {
		float2 prev = eval(t0).pos;
		for (int i=0; i<res; ++i) {
			float t = lerp(t0, t1, (float)(i+1) / res);

			auto bez = eval(t);
			
			if (i < res-1) {
				g_dbgdraw.line(float3(prev, z), float3(bez.pos, z), col);
			}
			else {
				g_dbgdraw.arrow(view, float3(prev, z), float3(bez.pos - prev, 0), 1, col);
			}

			prev = bez.pos;
		}
	}
};
struct Bezier4 {
	float2 a;
	float2 b;
	float2 c;
	float2 d;

	BezierRes eval (float t) const {
		//float2 ab = lerp(a, b, t);
		//float2 bc = lerp(b, c, t);
		//float2 cd = lerp(c, d, t);
		//
		//float2 abc = lerp(ab, bc, t);
		//float2 bcd = lerp(bc, cd, t);
		//
		//return lerp(abc, bcd, t);

		//float t2 = t*t;
		//float t3 = t2*t;
		//
		//float _3t1 = 3.0f*t;
		//float _3t2 = 3.0f*t2;
		//float _6t2 = 6.0f*t2;
		//float _3t3 = 3.0f*t3;
		//
		//float ca = 1.0f -_3t1 +_3t2   -t3;
		//float cb =       _3t1 -_6t2 +_3t3;
		//float cc =             _3t2 -_3t3;
		//float cd =                     t3;
		//
		//return (ca*a + cb*b) + (cc*c + cd*d);
		
		float2 c0 = a;                   // a
		float2 c1 = 3 * (b - a);         // (-3a +3b)t
		float2 c2 = 3 * (a + c) - 6*b;   // (3a -6b +3c)t^2
		float2 c3 = 3 * (b - c) - a + d; // (-a +3b -3c +d)t^3

		float t2 = t*t;
		float t3 = t2*t;
		
		float2 value = c3*t3     + c2*t2    + c1*t + c0; // f(t)
		float2 deriv = c3*(t2*3) + c2*(t*2) + c1;        // f'(t)
		float2 accel = c3*(t*6)  + c2*2;                 // f''(t)
		
		float denom = deriv.x*deriv.x + deriv.y*deriv.y;
		float curv = (deriv.x*accel.y - accel.x*deriv.y) / (denom * sqrt(denom)); // denom^(3/2)
		
		return { value, deriv, curv };
	}
};

struct Line {
	float3 a, b;
};

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
struct NullableVariant {
	typedef std::monostate null_t;
	std::variant<null_t, TYPES...> var;

	template <typename T>
	NullableVariant (T ref) {
		assert((bool)ref); // Assert that normal references are never null (should have assigned nullptr instead)
		var = ref;
	}

	template<>
	NullableVariant (nullptr_t ref) {
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

	operator bool () const { return var.index() != 0; }
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

inline void draggable (Input& I, View3D& view, float2* pos, float r) {
	// use ptr to identiy if and what we are dragging, to allow using this function with any number of items
	static float2* dragging_ptr = nullptr;
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
		drag_offs = hit_point - *pos;
		dragging_ptr = pos;
	}
	if (dragging_ptr == pos) {
		if (hit && I.buttons[MOUSE_BUTTON_LEFT].is_down) {
			*pos = hit_point - drag_offs;
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
