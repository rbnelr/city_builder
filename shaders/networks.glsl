#version 430
#include "common.glsl"
#include "gbuf.glsl"

// TODO: use instancing

VS2FS Vertex {
	vec3 pos;
	vec3 norm;
	vec3 tang;
	vec2 uv;
	flat int tex_id;
} v;

#ifdef _VERTEX
	layout(location = 0) in vec3  pos;
	layout(location = 1) in vec3  norm;
	layout(location = 2) in vec3  tang;
	layout(location = 3) in vec2  uv;
	layout(location = 4) in int   tex_id;
	
	void main () {
		gl_Position = view.world2clip * vec4(pos, 1.0);
		
		v.pos = pos;
		
		vec2 world_uv = pos.xy * 0.4;
		if (tex_id < 0) {
			v.tex_id = -tex_id;
			v.uv = world_uv;
			v.norm = vec3(0,0,1);
			v.tang = vec3(1,0,0);
		}
		else {
			v.tex_id = tex_id;
			v.uv = uv;
			v.norm = norm;
			v.tang = tang;
		}
		
		//if (tex_id == 14)
		//	dbgdraw_visualize_normal_tangent(v.pos, v.norm, v.tang);
	}
#endif
#ifdef _FRAGMENT
	uniform vec4 pbr = vec4(0.75, 0.0, 0,1);
	
	GBUF_OUT
	void main () {
		vec3 norm = normal_map(v.norm, v.tang,
		           bindless_tex_scaled(v.tex_id+1, v.uv).rgb );
		vec3 col = bindless_tex_scaled(v.tex_id  , v.uv).rgb;
		
		frag_col   = vec4(col, 1.0);
		frag_emiss = vec4(0,0,0,1);
		frag_norm  = vec4(norm, 1.0);
		frag_pbr   = pbr;
	}
#endif
