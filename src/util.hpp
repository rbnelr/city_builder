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
