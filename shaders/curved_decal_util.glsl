//#include "common.glsl"

float cross2d (vec2 l, vec2 r) {
	return l.x * r.y - l.y * r.x;
}

// adapted from https://iquilezles.org/articles/ibilinear/
bool uv_from_corners (vec2 a, vec2 b, vec2 c, vec2 d, vec2 p, out vec2 uv) {
	vec2 ab = b-a;
	vec2 ac = c-a;
	vec2 h = a-b+d-c;
	vec2 pa = p-a;
			
	float k2 = cross2d(h, ac);
	float k1 = cross2d(ab, ac) + cross2d(pa, h);
	float k0 = cross2d(pa, ab);
	
	float u;
	float v;
	
	if (abs(k2) < 0.001) {
		v = -k0 / k1;
		u = (pa.x*k1 + ac.x*k0) / (ab.x*k1 - h.x*k0);

		// No idea why this condition works, but doing this causes just the front side to be displayed!
		if (k0 > 0.0 || k1 <= 0.0)
			return false;
	}
	else {
		float p = -0.5f * k1 / k2;
		float q = k0 / k2;
		
		float sqr = p*p - q;
		if (sqr < 0.0)
			return false;
		sqr = sqrt(sqr);

		// No idea why this condition works, but doing this causes just the front side to be displayed!
		v = k2 < 0.0 ? p - sqr : p + sqr;
		u = (pa.x - ac.x*v) / (ab.x + h.x*v);
	}
	
	if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0)
		return false;
	
	uv = vec2(u,v);
	return true;
}
bool uv_from_corners_with_height (vec3 a, vec3 b, vec3 c, vec3 d, float h, vec3 p, out vec3 uv) {
	if (uv_from_corners(a.xy, b.xy, c.xy, d.xy, p.xy, uv.xy)) {
		// reconstruct height and compare
		float z = mix( mix(a.z,b.z, uv.x), mix(c.z,d.z, uv.y), uv.y);
		if (p.z > z && p.z < z+h) {
			uv.z = (p.z - z) / h;
			return true;
		}
	}
	return false;
}

VS2FS Vertex {
	// corners of quad region
	flat vec3 a, b, c, d; // back left, back right, front left, front right
	flat float height;
	flat vec2 local_uv_range;
	flat float uv_len;
	flat int  tex;
	flat vec4 col0;
	flat vec4 col1;
} v;

#ifdef _VERTEX
	layout(location =  0) in vec3  mesh_pos;
	
	layout(location =  1) in vec3  pos0;
	layout(location =  2) in vec3  pos1;
	layout(location =  3) in vec3  right0;
	layout(location =  4) in vec3  right1;
	layout(location =  5) in float height;
	layout(location =  6) in float uv0;
	layout(location =  7) in float uv1;
	layout(location =  8) in float uv_len;
	layout(location =  9) in int   tex;
	layout(location = 10) in vec4  col0;
	layout(location = 11) in vec4  col1;

	void main () {
		vec3 up = vec3(0,0, height * 0.5);
		
		v.a = pos0 - right0 - up;
		v.b = pos0 + right0 - up;
		v.c = pos1 - right1 - up;
		v.d = pos1 + right1 - up;
		
		v.height = height;
		v.local_uv_range = vec2(uv0, uv1);
		v.uv_len = uv_len;
		v.tex    = tex   ;
		v.col0   = col0  ;
		v.col1   = col1  ;
		
		vec3 p0 = mix(v.a, v.b, mesh_pos.x);
		vec3 p1 = mix(v.c, v.d, mesh_pos.x);
		vec3 pos = mix(p0, p1, mesh_pos.y);
		pos += mesh_pos.z * up*2.0;
		
		gl_Position = view.world2clip * vec4(pos, 1.0);
	}
#endif
#ifdef _FRAGMENT
	#define GBUF_IN 1
	#include "gbuf.glsl"
	
	uniform float fade_strength = 0.5;
	
	bool curved_decal (out vec2 uv, out vec4 col, out vec3 pos_world) {
		if (!decode_gbuf_pos(pos_world))
			return false;
		
		vec3 local_coords;
		if (!uv_from_corners_with_height(v.a, v.b, v.c, v.d, v.height, pos_world, local_coords))
			return false;
		
		// uvs of current block, with V coming from vertex data
		uv = vec2(local_coords.x, mix(v.local_uv_range.x, v.local_uv_range.y, local_coords.y));
		
		// reconstruct color
		col = mix(v.col0, v.col1, local_coords.y);
		
		// fade out at top and bottom bounds of decal volume
		float fade = smoothstep(0.0, 1.0, map(abs(local_coords.z * 2.0 - 1.0), 1.0, 0.999 - fade_strength));
		col.a *= fade;
		
		return true;
	}
#endif
