#version 430
#include "common.glsl"
#include "gbuf.glsl"

VS2FS Vertex {
	vec3 world_pos;
	vec3 world_normal;
	vec2 uv;
	
	flat vec3 col;
	flat int tex_id;
} v;

#ifdef _VERTEX

layout(location = 0) in vec3  mesh_pos;
layout(location = 1) in vec3  mesh_normal;
layout(location = 2) in vec2  mesh_uv;
//layout(location = 3) in int   mesh_id;
layout(location = 4) in int   tex_id;
layout(location = 5) in vec3  instance_pos;
layout(location = 6) in float instance_rot;
layout(location = 7) in vec3  instance_col;

void main () {
	mat3 rot_mat = mat_rotateZ(instance_rot);
	
	v.world_pos    = rot_mat * mesh_pos + instance_pos;
	v.world_normal = rot_mat * mesh_normal;
	
	v.uv           = mesh_uv;
	v.col          = instance_col;
	v.tex_id       = tex_id;
	
	gl_Position = view.world2clip * vec4(v.world_pos, 1.0);
}

#endif

#ifdef _FRAGMENT
	GBUF_OUT
	void main () {
		vec4 col = texture(bindless_tex(v.tex_id), v.uv);
		vec3 TRM = texture(bindless_tex(v.tex_id+1), v.uv).rgb;
		
		//col.rgb += mix(vec3(1.0), v.col, TRM.r);
		col.rgb += (vec3(1.0,.5,0.0) * TRM.r) * 2.2;
		
		frag_col = col;
		frag_norm = vec4(v.world_normal, 1.0);
		frag_pbr  = vec4(TRM.gb, 0,1);
	}
#endif
