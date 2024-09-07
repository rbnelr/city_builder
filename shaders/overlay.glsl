#version 430
#include "common.glsl"

vec3 bezier (float t, vec3 a, vec3 b, vec3 c, vec3 d, out vec3 grad) {
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

#ifdef _VERTEX
	layout(location = 0) in float mesh_t;
	layout(location = 1) in vec3 inst_bez_a;
	layout(location = 2) in vec3 inst_bez_b;
	layout(location = 3) in vec3 inst_bez_c;
	layout(location = 4) in vec3 inst_bez_d;
	layout(location = 5) in vec2 inst_bez_size;
	layout(location = 6) in vec4 inst_col;

	out Vertex {
		vec3 pos;
		vec3 right;
		vec3 up;
		vec4 col;
	} v;

	void main () {
		vec3 bez_grad;
		v.pos = bezier(mesh_t, inst_bez_a, inst_bez_b, inst_bez_c, inst_bez_d, bez_grad);
		
		v.right = normalize(vec3(bez_grad.y, -bez_grad.x, bez_grad.z)) * inst_bez_size.x * 0.5;
		v.up = vec3(0,0,1) * inst_bez_size.y * 0.5;
		
		v.col = inst_col;
	}
#endif

#ifdef _GEOMETRY
	layout(lines) in;
	layout(triangle_strip, max_vertices = 24) out; // 4*6

	in Vertex {
		vec3 pos;
		vec3 right;
		vec3 up;
		vec4 col;
	} v[];
	out Geom {
		vec4 col;
	} g;
	
	/* vertex order:
		{0,0,0},
		{1,0,0},
		{1,1,0},
		{0,1,0},
		{0,0,1},
		{1,0,1},
		{1,1,1},
		{0,1,1},
	*/
	
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
		
		cube_verts[0] = v[0].pos - v[0].right - v[0].up;
		cube_verts[1] = v[0].pos + v[0].right - v[0].up;
		cube_verts[2] = v[1].pos + v[1].right - v[1].up;
		cube_verts[3] = v[1].pos - v[1].right - v[1].up;
		cube_verts[4] = v[0].pos - v[0].right + v[0].up;
		cube_verts[5] = v[0].pos + v[0].right + v[0].up;
		cube_verts[6] = v[1].pos + v[1].right + v[1].up;
		cube_verts[7] = v[1].pos - v[1].right + v[1].up;
		
		cube_uvs[0] = vec2(0,0);
		cube_uvs[1] = vec2(1,0);
		cube_uvs[2] = vec2(1,1);
		cube_uvs[3] = vec2(0,1);
		cube_uvs[4] = vec2(0,0);
		cube_uvs[5] = vec2(1,0);
		cube_uvs[6] = vec2(1,1);
		cube_uvs[7] = vec2(0,1);
		
		for (int face=0; face<6; ++face) {
			for (int i=0; i<4; ++i) {
				int idx = cube_indices[face*4 + i];
				gl_Position = view.world2clip * vec4(cube_verts[idx], 1.0);
				//g.col = v[0].col;
				g.col = vec4(cube_uvs[idx], 0,1);
				
				EmitVertex();
			}
			EndPrimitive();
		}
	}
#endif

#ifdef _FRAGMENT
	#include "gbuf.glsl"
	
	in Geom {
		vec4 col;
	} g;
	
	GBUF_OUT
	void main () {
		frag_col  = g.col;
		frag_emiss = vec4(0,0,0,1);
		frag_norm = vec4(0,0,1,1);
		frag_pbr = vec4(1,0,0,1);
	}
#endif
