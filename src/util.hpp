#pragma once
#include "common.hpp"

inline float angle2d (float2 dir) {
	return length_sqr(dir) > 0 ? atan2f(dir.y, dir.x) : 0;
}

struct BezierRes {
	float2 pos;
	float2 vel;   // velocity (over bezier t)
	float  curv;  // curvature (delta angle over dist along curve)
};
inline BezierRes bezier3 (float t, float2 a, float2 b, float2 c) {
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
inline BezierRes bezier4 (float t, float2 a, float2 b, float2 c, float2 d) {
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

struct Bezier3 {
	float3 a, b, c;
};

inline bool line_line_intersect (float2 a, float2 b, float2 c, float2 d, float2* out_point) {
	// https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
	float numer = (a.x-c.x)*(c.y-d.y) - (a.y-c.y)*(c.x-d.x);
	float denom = (a.x-b.x)*(c.y-d.y) - (a.y-b.y)*(c.x-d.x);
	if (denom == 0)
		return false; // parallel, either overlapping (numer == 0) or not
	float u = numer / denom;
	*out_point = a + u*(b-a);
	return true; // always intersect for now
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

inline bool ray_box_intersection (float2 a, float2 da, float2 b, float2 db, float r, float* out_u) {
	float len_b = length(db);

	db = normalize(db);
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
		float t0 = offs_x0 / dx;
		float t1 = offs_x1 / dx;
		tx0 = min(t0, t1);
		tx1 = max(t0, t1);
	}
	else {
		if (offs_x0 > 0.0f || offs_x1 < 0.0f)
			return false; // miss
	}
	
	float ty0 = -INF, ty1 = +INF;
	if (dy != 0.0f) {
		float t0 = offs_y0 / dy;
		float t1 = offs_y1 / dy;
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
	void highlight_selected () {
		g_dbgdraw.wire_circle(pos, radius, lrgba(col,1), 32);
	}
};

inline void dbg_draw_boxy_line (float3 a, float3 b, float r, lrgba col) {
	
	float2 offs = b - a;
	float2 forw = normalizesafe(offs) * r;
	float2 right = float2(forw.y, -forw.x);

	g_dbgdraw.line(a + float3(-forw -right, 0.0f), a + float3(-forw +right, 0.0f), col);
	g_dbgdraw.line(a + float3(-forw +right, 0.0f), b + float3(+forw +right, 0.0f), col);
	g_dbgdraw.line(b + float3(+forw +right, 0.0f), b + float3(+forw -right, 0.0f), col);
	g_dbgdraw.line(b + float3(+forw -right, 0.0f), a + float3(-forw -right, 0.0f), col);
}
