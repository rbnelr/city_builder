#pragma once
#include "common.hpp"

template <typename VEC>
struct BezierRes {
	VEC pos;
	VEC vel;   // velocity (over bezier t)
	float  curv;  // curvature (delta angle over dist along curve)
};

template <typename VEC=float3>
struct Bezier3 {
	SERIALIZE(Bezier3, a, b, c)

	VEC a;
	VEC b;
	VEC c;

	BezierRes<VEC> eval (float t) const {
		VEC c0 = a;           // a
		VEC c1 = 2 * (b - a); // (-2a +2b)t
		VEC c2 = a - 2*b + c; // (a -2b +c)t^2
		
		float t2 = t*t;

		VEC value = c2*t2    + c1*t + c0; // f(t)
		VEC deriv = c2*(t*2) + c1;        // f'(t)

		return { value, deriv };
	}

	BezierRes<VEC> eval_with_curv (float t) const {
		//VEC ab = lerp(a, b, t);
		//VEC bc = lerp(b, c, t);
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

		VEC c0 = a;           // a
		VEC c1 = 2 * (b - a); // (-2a +2b)t
		VEC c2 = a - 2*b + c; // (a -2b +c)t^2
		
		float t2 = t*t;

		VEC value = c2*t2    + c1*t + c0; // f(t)
		VEC deriv = c2*(t*2) + c1;        // f'(t)
		VEC accel = c2*2;                 // f''(t)

		
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

		// curvature is positive when the curve curves CCW, since atan grows CCW
		float denom = deriv.x*deriv.x + deriv.y*deriv.y;
		float curv = (deriv.x*accel.y - accel.x*deriv.y) / (denom * sqrt(denom)); // denom^(3/2)

		return { value, deriv, curv };
	}

	// Optimization if compier is not smart enough to optimize loop invariant
	struct Coefficients {
		VEC c0;
		VEC c1;
		VEC c2;
	};
	Coefficients get_coeff () {
		Coefficients co; 
		co.c0 = a;           // a
		co.c1 = 2 * (b - a); // (-2a +2b)t
		co.c2 = a - 2*b + c; // (a -2b +c)t^2
		return co;
	}
	VEC eval_value (Coefficients const& co, float t) {
		float t2 = t*t;
		return co.c2*t2 + co.c1*t + co.c0;
	}
	
	// it's faster for check_conflict because compiler is dum dum
	// COLLISION_STEPS is small and constant, so loop is unrollen and ca, cb, cc become constants, which compiler does not catch
	VEC eval_value_fast_for_const_t (float t) const {
		float t2 = t*t;
		
		float _2t1 = 2.0f*t;
		float _2t2 = 2.0f*t2;
		
		float ca = 1.0f -_2t1   +t2;
		float cb =       _2t1 -_2t2;
		float cc =               t2;
		
		VEC v;
		v.x = ca*a.x + cb*b.x + cc*c.x;
		v.y = ca*a.y + cb*b.y + cc*c.y;
		return v;
	}

	float approx_len (int steps) {
		auto co = get_coeff();
		VEC prev = a;

		float len = 0;
		for (int i=0; i<steps; ++i) {
			float t = (float)(i+1) * (1.0f / steps);
			VEC pos = eval_value(co, t);
			len += length(pos - prev);
			prev = pos;
		}

		return len;
	}
};

template <typename VEC=float3>
struct Bezier4 {
	SERIALIZE(Bezier4, a, b, c, d)

	VEC a;
	VEC b;
	VEC c;
	VEC d;

	BezierRes<VEC> eval (float t) const {
		//VEC ab = lerp(a, b, t);
		//VEC bc = lerp(b, c, t);
		//VEC cd = lerp(c, d, t);
		//
		//VEC abc = lerp(ab, bc, t);
		//VEC bcd = lerp(bc, cd, t);
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
		
		VEC c0 = a;                   // a
		VEC c1 = 3 * (b - a);         // (-3a +3b)t
		VEC c2 = 3 * (a + c) - 6*b;   // (3a -6b +3c)t^2
		VEC c3 = 3 * (b - c) - a + d; // (-a +3b -3c +d)t^3

		float t2 = t*t;
		float t3 = t2*t;
		
		VEC value = c3*t3     + c2*t2    + c1*t + c0; // f(t)
		VEC deriv = c3*(t2*3) + c2*(t*2) + c1;        // f'(t)
		VEC accel = c3*(t*6)  + c2*2;                 // f''(t)
		
		float denom = deriv.x*deriv.x + deriv.y*deriv.y;
		float curv = (deriv.x*accel.y - accel.x*deriv.y) / (denom * sqrt(denom)); // denom^(3/2)
		
		return { value, deriv, curv };
	}
};

template <typename T>
inline void _dbg_draw_bez (T const& bez, int res, lrgba col, float t0=0, float t1=1) {
	auto prev = bez.eval(t0).pos;
	for (int i=0; i<res; ++i) {
		float t = lerp(t0, t1, (float)(i+1) / res);

		auto val = bez.eval(t);
			
		if (i < res-1) {
			g_dbgdraw.line(prev, val.pos, col);
		}
		else {
			g_dbgdraw.arrow(prev, val.pos - prev, 1, col);
		}

		prev = val.pos;
	}
}
inline void dbg_draw_bez (Bezier3<float3> const& bez, int res, lrgba col, float t0=0, float t1=1) {
	_dbg_draw_bez(bez, res, col, t0, t1);
}
inline void dbg_draw_bez (Bezier3<float4> const& bez, int res, lrgba col, float t0=0, float t1=1) {
	_dbg_draw_bez(bez, res, col, t0, t1);
}