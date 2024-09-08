//#include "common.glsl"

vec3 bezier (vec3 a, vec3 b, vec3 c, vec3 d, float t, out vec3 grad) {
	vec3 c0 = a;                     // a
	vec3 c1 = 3.0 * (b - a);         // (-3a +3b)t
	vec3 c2 = 3.0 * (a + c) - 6.0*b; // (3a -6b +3c)t^2
	vec3 c3 = 3.0 * (b - c) - a + d; // (-a +3b -3c +d)t^.03

	float t2 = t*t;
	float t3 = t2*t;
	
	vec3 value = c3*t3       + c2*t2      + c1*t + c0; // f(t)
	grad       = c3*(t2*3.0) + c2*(t*2.0) + c1;        // f'(t)
	return value;
}

bool uv_from_corners (vec2 a, vec2 b, vec2 c, vec2 d, vec2 p, out vec2 uv) {
	vec2 ab = b - a;
	vec2 ac = c - a;
	vec2 ad = d - a;
	
	vec2 H = ad - ab - ac;
	vec2 O = p - a;

	float u, v;
	{
		float a = ab.y*H.x - ab.x*H.y;
		float b = H.y*O.x - H.x*O.y + ab.y*ac.x - ab.x*ac.y;
		float c = O.x*ac.y - O.y*ac.x;
		if (abs(a) < 0.0001) { // b*u + c = 0
			u = -c / b;
			
			// No idea why this condition works, but doing this causes just the front side to be displayed!
			if (c < 0.0 || b >= 0.0)
				return false;
		}
		else { // a*u^2 + b*u + c = 0
			float p = -0.5 * b / a;
			float q = c / a;

			float sqr = p*p - q;
			if (sqr < 0.0)
				return false;
			sqr = sqrt(sqr);
			// No idea why this condition works, but doing this causes just the front side to be displayed!
			u = a > 0.0 ? p - sqr : p + sqr;
		}
		if (u < 0.0 || u > 1.0)
			return false;
	}
	{
		// The fact that the conditions are flipped tells me a/b/c are probably flipped (the divisions cancel out the sign, but the condition flips)
		// Seems like deriving and then trial&erroring the conditions caused the formula to be reversed with respect to all the substractions?
		float a = ac.y*H.x - ac.x*H.y;
		float b = H.y*O.x - H.x*O.y + ac.y*ab.x - ac.x*ab.y;
		float c = O.x*ab.y - O.y*ab.x;

		if (abs(a) < 0.0001) {
			v = -c / b;
			
			if (c > 0.0 || b <= 0.0)
				return false;
		}
		else {
			float p = -0.5f * b / a;
			float q = c / a;
	
			float sqr = p*p - q;
			if (sqr < 0.0)
				return false;
			sqr = sqrt(sqr);

			v = a < 0.0 ? p - sqr : p + sqr;
		}
		if (v < 0.0 || v > 1.0)
			return false;
	}
	
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

struct Vertex {
	vec3 pos;
	vec3 right;
	float k; // dist along curve (not bezier t!)
	float height;
	vec4 uv_remap;
	vec4 col;
	int tex;
};

#ifdef _VERTEX
	layout(location = 0) in float mesh_t;
	layout(location = 1) in vec3 inst_bez_a;
	layout(location = 2) in vec3 inst_bez_b;
	layout(location = 3) in vec3 inst_bez_c;
	layout(location = 4) in vec3 inst_bez_d;
	layout(location = 5) in vec4 inst_size;
	layout(location = 6) in vec4 inst_uv_remap;
	layout(location = 7) in vec4 inst_col;
	layout(location = 8) in int  inst_tex;

	out Vertex v;

	void main () {
		float t = mix(inst_size.x, inst_size.y, mesh_t);
		vec3 bez_grad;
		v.pos = bezier(inst_bez_a, inst_bez_b, inst_bez_c, inst_bez_d, t, bez_grad);
		v.right = normalize(vec3(bez_grad.y, -bez_grad.x, bez_grad.z)) * inst_size.z * 0.5;
		v.k = mesh_t;
		v.height = inst_size.w;
		v.uv_remap = inst_uv_remap;
		v.col = inst_col;
		v.tex = inst_tex;
	}
#endif

struct Geom {
	vec2 k_range;
	// corners of quad region
	flat vec3 a, b, c, d; // back left, back right, front left, front right
	flat float height;
	flat vec4 uv_remap;
	flat vec4 col;
	flat int  tex;
};

#ifdef _GEOMETRY
	layout(lines) in;
	layout(triangle_strip, max_vertices = 24) out; // 4*6

	in Vertex v[];
	out Geom g;
	
	// D---C
	// | / |
	// A---B
	// Specified as B,C,A,D for trangle strip
	const int cube_indices[] = {
		1,5,0,4,
		2,6,1,5,
		3,7,2,6,
		0,4,3,7,
		5,6,4,7,
		2,1,3,0,
	};
	
	void main () {
		vec3 cube_verts[8];
		vec2 cube_uvs[8];
		
		vec3 up = vec3(0,0, v[0].height * 0.5);
		
		Geom geom;
		geom.k_range = vec2(v[0].k, v[1].k);
		geom.a = v[0].pos - v[0].right - up;
		geom.b = v[0].pos + v[0].right - up;
		geom.c = v[1].pos - v[1].right - up;
		geom.d = v[1].pos + v[1].right - up;
		geom.height = v[0].height;
		geom.uv_remap = v[0].uv_remap;
		geom.col = v[0].col;
		geom.tex = v[0].tex;
		
		cube_verts[0] = geom.a;
		cube_verts[1] = geom.b;
		cube_verts[2] = geom.d;
		cube_verts[3] = geom.c;
		cube_verts[4] = geom.a + up*2.0;
		cube_verts[5] = geom.b + up*2.0;
		cube_verts[6] = geom.d + up*2.0;
		cube_verts[7] = geom.c + up*2.0;
		
		for (int face=0; face<6; ++face) {
			for (int i=0; i<4; ++i) {
				int idx = cube_indices[face*4 + i];
				gl_Position = view.world2clip * vec4(cube_verts[idx], 1.0);
				g = geom;
				
				EmitVertex();
			}
			EndPrimitive();
		}
	}
#endif

#ifdef _FRAGMENT
	#define GBUF_IN 1
	#include "gbuf.glsl"
	
	in Geom g;
	
	uniform float fade_strength = 0.5;
	
	bool curved_decal (out vec2 uv, out float fade, out vec3 pos_world) {
		if (!decode_gbuf_pos(pos_world))
			return false;
		
		vec3 local_coords;
		if (!uv_from_corners_with_height(g.a, g.b, g.c, g.d, g.height, pos_world, local_coords))
			return false;
		
		// map current block into correct distance along curve, and use that as uv
		uv = vec2(local_coords.x, mix(g.k_range.x, g.k_range.y, local_coords.y));
		// apply uv remap for this curve
		uv = mix(g.uv_remap.xy, g.uv_remap.zw, uv);
		
		// fade out at top and bottom bounds of decal volume
		fade = smoothstep(0.0, 1.0, map(abs(local_coords.z * 2.0 - 1.0), 1.0, 0.999 - fade_strength));
		
		return true;
	}
#endif
