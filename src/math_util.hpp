#pragma once
#include "engine/kisslib/kissmath.hpp"
#include "engine/kisslib/collision.hpp"
#include "engine/camera.hpp"
#include "bezier.hpp"

inline float angle2d (float2 dir) {
	return length_sqr(dir) > 0 ? atan2f(dir.y, dir.x) : 0;
}

inline float lrgb_luminance (lrgb const& col) {
	return dot(col, lrgb(0.2126f, 0.7152f, 0.0722f));
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

inline int intersect_ray_zcircle (Ray const& ray, float3 center, float r, float* hit_t=nullptr, float2* hit_point=nullptr) {
	float t;
	if (!intersect_ray_zplane(ray, center.z, &t))
		return -1;
	
	float2 xy = (float2)ray.pos + (float2)ray.dir * t;

	if (hit_t) *hit_t = t;
	if (hit_point) *hit_point = xy;

	float dist_sqr = length_sqr(xy - (float2)center);
	if (dist_sqr > r*r)
		return 0;

	return 1;
}

inline bool line_line_intersect (float2 const& a, float2 const& ab, float2 const& c, float2 const& cd, float2* out_point) {
	// https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
	float2 ac = c - a;
	float numer = ac.x * cd.y - ac.y * cd.x;
	float denom = ab.x * cd.y - ab.y * cd.x;
	if (denom == 0)
		return false; // parallel, either overlapping (numer == 0) or not
	float u = numer / denom;
	*out_point = a + u*ab;
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

inline bool intersect_rect_ray (float3 pos, float3 forw, float3 right, Ray const& ray, float* hit_dist) {
	float3 up = normalize(cross(right, forw));

	float3 rel = pos - ray.pos;

	float denom = dot(up, ray.dir);
	if (denom == 0.0f) return false;

	float t = dot(up, rel) / denom;
	if (t < 0.0f) return false;
	
	float3 hit_pos_rel = ray.dir * t - rel;

	float y = dot(hit_pos_rel, normalize(forw)) / length(forw);
	float x = dot(hit_pos_rel, normalize(right)) / length(right);

	if ( y < 0.0f || y > 1.0f ||
	     x < 0.0f || x > 1.0f )
		return false;

	*hit_dist = t;
	return true;
}

inline float3 ndc2world (float4x4 const& clip2world, float3 world) {
/* Why this works:
	clip.xyzw = world2clip * (world.xyz, 1)
	ndc.xyz = clip.xyz / clip.w
	=>
	clip2world * clip.xyz = (world.xyz, 1)
	=>
	dot(clip2world.row[0], clip.xyzw) = world.x
	dot(clip2world.row[1], clip.xyzw) = world.y
	dot(clip2world.row[2], clip.xyzw) = world.z
	dot(clip2world.row[3], clip.xyzw) = 1
	
	clip.xyz = ndc.xyz * clip.w => clip.xyzw = (ndc.xyz, 1) * clip.w
	=>
	dot(clip2world.row[3], (ndc.xyz, 1) * clip.w) = 1
	=>
	dot(clip2world.row[0], (ndc.xyz, 1)) = world.x * clip.w
	dot(clip2world.row[1], (ndc.xyz, 1)) = world.y * clip.w
	dot(clip2world.row[2], (ndc.xyz, 1)) = world.z * clip.w
	dot(clip2world.row[3], (ndc.xyz, 1)) = 1 * clip.w
	=>
	clip2world * (ndc.xyz, 1) = (world.xyz * clip.w, clip.w)
	=>
	tmp.xyzw = clip2world * (ndc.xyz, 1)
	world.xyz = tmp.xyz / tmp.w
*/

	float4 tmp = clip2world * float4(world, 1);
	return float3(tmp) / tmp.w;
}

inline void dbgdraw_frustrum (float4x4 const& clip2world, lrgba const& col) {
	
	float depth0 = -1, depth1 = 1.0f - 0.00001f; // TODO: test that this actually works?
#if OGL_USE_REVERSE_DEPTH
	if (ogl::reverse_depth) {
		depth0 = 1; depth1 = 0.00001f; // handle potential infine far plane (reverse depth)
	}
#endif

	float3 p  = ndc2world(clip2world, float3(0,0,+1));
	float3 n0 = ndc2world(clip2world, float3(-1,-1, depth0));
	float3 n1 = ndc2world(clip2world, float3(+1,-1, depth0));
	float3 n2 = ndc2world(clip2world, float3(+1,+1, depth0));
	float3 n3 = ndc2world(clip2world, float3(-1,+1, depth0));
	float3 f0 = ndc2world(clip2world, float3(-1,-1, depth1));
	float3 f1 = ndc2world(clip2world, float3(+1,-1, depth1));
	float3 f2 = ndc2world(clip2world, float3(+1,+1, depth1));
	float3 f3 = ndc2world(clip2world, float3(-1,+1, depth1));
		
	float3 hat = ndc2world(clip2world, float3(0,1.5f,+1));

	// close rect
	g_dbgdraw.line(n0, n1, col);
	g_dbgdraw.line(n1, n2, col);
	g_dbgdraw.line(n2, n3, col);
	g_dbgdraw.line(n3, n0, col);
	// little hat
	g_dbgdraw.line(n2, hat, col);
	g_dbgdraw.line(hat, n3, col);
	// lines into possibly infine far plane 
	g_dbgdraw.line(n0, f0, col);
	g_dbgdraw.line(n1, f1, col);
	g_dbgdraw.line(n2, f2, col);
	g_dbgdraw.line(n3, f3, col);
	// far rect
	g_dbgdraw.line(f0, f1, col * lrgba(1,1,1,0.25f));
	g_dbgdraw.line(f1, f2, col * lrgba(1,1,1,0.25f));
	g_dbgdraw.line(f2, f3, col * lrgba(1,1,1,0.25f));
	g_dbgdraw.line(f3, f0, col * lrgba(1,1,1,0.25f));
}

//#include "immintrin.h"

struct View_Frustrum {
	// plane equations extracted from world2clip matrix
	// a,b,c,d -> abc is normalized normal vector pointing into frustrum
	// a*x + b*y + c*z + d == 0 is the plane equation (all in world space) so > 0 means inside
	// near, left, right, bototm, top planes
	float4 planes[5];

	//__m256 a, b, c, d;
};
inline View_Frustrum clac_view_frustrum (View3D const& view) {
	auto const& m = view.world2clip;
	float4 L = m.get_row(3) + m.get_row(0);
	float4 R = m.get_row(3) - m.get_row(0);
	float4 B = m.get_row(3) + m.get_row(1);
	float4 T = m.get_row(3) - m.get_row(1);
	
	float4 N, F;
#if OGL_USE_REVERSE_DEPTH
	if (ogl::reverse_depth) {
		N = m.get_row(3) - m.get_row(2);
		F =                m.get_row(2);
	} else
#endif
	{
		N = m.get_row(3) + m.get_row(2);
		F = m.get_row(3) - m.get_row(2);
	}

	L /= length(float3(L));
	R /= length(float3(R));
	B /= length(float3(B));
	T /= length(float3(T));
	N /= length(float3(N));
	F /= length(float3(F));

	View_Frustrum frust;
	frust.planes[0] = N;
	frust.planes[1] = L;
	frust.planes[2] = R;
	frust.planes[3] = B;
	frust.planes[4] = T;

	//frust.a = _mm256_set_ps(0,0,0, T.x, B.x, R.x, L.x, N.x);
	//frust.b = _mm256_set_ps(0,0,0, T.y, B.y, R.y, L.y, N.y);
	//frust.c = _mm256_set_ps(0,0,0, T.z, B.z, R.z, L.z, N.z);
	//frust.d = _mm256_set_ps(0,0,0, -T.w, -B.w, -R.w, -L.w, -N.w);
	
#if 0
	// Draw frustrum and plane normals for debugging
	float depth0 = -1, depth1 = 1.0f - 0.00001f;
	#if OGL_USE_REVERSE_DEPTH
	if (ogl::reverse_depth) {
		depth0 = 1; depth1 = 0.00001f; // handle potential infine far plane (reverse depth)
	}
	#endif

	float3 a = ndc2world(view.clip2world, float3(-1,-1, depth0));
	float3 b = ndc2world(view.clip2world, float3(+1,-1, depth0));
	float3 c = ndc2world(view.clip2world, float3(+1,+1, depth0));
	float3 d = ndc2world(view.clip2world, float3(-1,+1, depth0));
	float3 e = ndc2world(view.clip2world, float3(-1,-1, depth1));
	float3 f = ndc2world(view.clip2world, float3(+1,-1, depth1));
	float3 g = ndc2world(view.clip2world, float3(+1,+1, depth1));
	float3 h = ndc2world(view.clip2world, float3(-1,+1, depth1));

	g_dbgdraw.line(a, b, lrgba(1,1,0,1));
	g_dbgdraw.line(b, c, lrgba(1,1,0,1));
	g_dbgdraw.line(c, d, lrgba(1,1,0,1));
	g_dbgdraw.line(d, a, lrgba(1,1,0,1));
	g_dbgdraw.line(e, f, lrgba(1,1,0,1));
	g_dbgdraw.line(f, g, lrgba(1,1,0,1));
	g_dbgdraw.line(g, h, lrgba(1,1,0,1));
	g_dbgdraw.line(h, e, lrgba(1,1,0,1));
	g_dbgdraw.line(a, e, lrgba(1,1,0,1));
	g_dbgdraw.line(b, f, lrgba(1,1,0,1));
	g_dbgdraw.line(c, g, lrgba(1,1,0,1));
	g_dbgdraw.line(d, h, lrgba(1,1,0,1));
	
	g_dbgdraw.arrow(a, L, 0.1f, lrgba(1,0,0,1));
	g_dbgdraw.arrow(c, R, 0.1f, lrgba(1,1,0,1));
	g_dbgdraw.arrow(a, B, 0.1f, lrgba(0,0,1,1));
	g_dbgdraw.arrow(c, T, 0.1f, lrgba(0,1,1,1));
	g_dbgdraw.arrow(a, N, 0.1f, lrgba(1,0,1,1));
	//g_dbgdraw.arrow(e, F, 0.1f, lrgba(1,0.5f,0.5f,1));
#endif
	return frust;
}

#if 1
inline bool frustrum_cull_aabb (View_Frustrum const& frust, AABB3 const& aabb) {
	float3 lo = aabb.lo;
	float3 hi = aabb.hi;
	for (auto& pl : frust.planes) {
		
	#if 0
		// if any aabb corner lies inside frustrum, continue testing other planes
		if (pl.x*lo.x + pl.y*lo.y + pl.z*lo.z >= -pl.w) continue;
		if (pl.x*hi.x + pl.y*lo.y + pl.z*lo.z >= -pl.w) continue;
		if (pl.x*lo.x + pl.y*hi.y + pl.z*lo.z >= -pl.w) continue;
		if (pl.x*hi.x + pl.y*hi.y + pl.z*lo.z >= -pl.w) continue;
		if (pl.x*lo.x + pl.y*lo.y + pl.z*hi.z >= -pl.w) continue;
		if (pl.x*hi.x + pl.y*lo.y + pl.z*hi.z >= -pl.w) continue;
		if (pl.x*lo.x + pl.y*hi.y + pl.z*hi.z >= -pl.w) continue;
		if (pl.x*hi.x + pl.y*hi.y + pl.z*hi.z >= -pl.w) continue;
	#else
		// slightly slower for some reason?, because original earlies out?
		float x = max(pl.x*lo.x, pl.x*hi.x);
		float y = max(pl.y*lo.y, pl.y*hi.y);
		float z = max(pl.z*lo.z, pl.z*hi.z);
		float max_sum = x + y + z;

		if (max_sum >= -pl.w) continue;
	#endif

		return true; // all aabb corners lie outside plane, safe to cull
	}

	return false; // aabb not outside any plane, not safe to cull (but false negative possible)
}
#else
// Chefskiss, nice 5-wide simd use
inline bool frustrum_cull_aabb (View_Frustrum const& frust, AABB3 const& aabb) {
	__m256 loX = _mm256_set1_ps(aabb.lo.x);
	__m256 loY = _mm256_set1_ps(aabb.lo.y);
	__m256 loZ = _mm256_set1_ps(aabb.lo.z);
	__m256 hiX = _mm256_set1_ps(aabb.hi.x);
	__m256 hiY = _mm256_set1_ps(aabb.hi.y);
	__m256 hiZ = _mm256_set1_ps(aabb.hi.z);

	__m256 x = _mm256_max_ps(_mm256_mul_ps(frust.a, loX), _mm256_mul_ps(frust.a, hiX));
	__m256 y = _mm256_max_ps(_mm256_mul_ps(frust.b, loY), _mm256_mul_ps(frust.b, hiY));
	__m256 z = _mm256_max_ps(_mm256_mul_ps(frust.c, loZ), _mm256_mul_ps(frust.c, hiZ));

	__m256 max_sum = _mm256_add_ps(_mm256_add_ps(x, y), z);
	
	__m256 cmp = _mm256_cmp_ps(max_sum, frust.d, _CMP_LT_OQ);
	return _mm256_movemask_ps(cmp) != 0; // any true => culled
}
#endif
