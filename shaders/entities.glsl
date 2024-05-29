#version 430
#include "common.glsl"
#include "gbuf.glsl"

VS2FS Vertex {
	vec3 world_pos;
	vec3 world_normal;
	vec2 uv;
	
	flat vec3 col;
} v;

#ifdef _VERTEX

layout(location = 0) in vec3  mesh_pos;
layout(location = 1) in vec3  mesh_normal;
layout(location = 2) in vec2  mesh_uv;
//layout(location = 3) in int   mesh_id;
layout(location = 4) in vec3  instance_pos;
layout(location = 5) in vec3  instance_rot;
layout(location = 6) in vec3  instance_col;

void main () {
	mat3 rot_mat = instance_euler_mat(instance_rot.z, instance_rot.y, instance_rot.x);
	
	v.world_pos    = rot_mat * mesh_pos + instance_pos;
	v.world_normal = rot_mat * mesh_normal;
	
	v.uv           = mesh_uv;
	v.col          = instance_col;
	
	gl_Position = view.world2clip * vec4(v.world_pos, 1.0);
}

#endif

#ifdef _FRAGMENT
	uniform sampler2D tex;
	
	GBUF_OUT
	void main () {
		frag_col = texture(tex, v.uv) * vec4(v.col, 1.0);
		frag_norm = vec4(v.world_normal, 1.0);
	}
#endif
