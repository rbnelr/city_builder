#version 430
#include "common.glsl"

VS2FS Vertex {
	vec3 pos;
	vec4 col;
} v;

#ifdef _VERTEX
	layout(location = 0) in vec3 mesh_pos;
	layout(location = 1) in vec4 mesh_col;
	
	void main () {
		vec4 pos_world = view.world2clip * vec4(mesh_pos, 1.0);
		gl_Position = pos_world;
		v.pos = pos_world.xyz;
		v.col = mesh_col;
	}
#endif
#ifdef _FRAGMENT
	#define GBUF_IN 1
	#include "gbuf.glsl"
	
	GBUF_OUT
	void main () {
		//vec3 gbuf_pos_world;
		//if (!decode_gbuf_pos(gbuf_pos_world))
		//	discard;
		
		frag_col  = v.col;
		frag_emiss = vec4(0,0,0,1);
		frag_norm = vec4(0,0,1,1);
		frag_pbr = vec4(1,0,0,1);
	}
#endif
