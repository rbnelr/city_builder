#version 430
#include "common.glsl"

struct Vertex {
	vec3 world_pos;
	vec3 world_normal;
	vec2 uv;
	vec3 col;
};
VS2FS

#ifdef _VERTEX

layout(location = 0) in vec3  mesh_pos;
layout(location = 1) in vec3  mesh_normal;
layout(location = 2) in vec2  mesh_uv;
//layout(location = 3) in int   mesh_id;
layout(location = 4) in vec3  instance_pos;
layout(location = 5) in float instance_rot;
layout(location = 6) in vec3  instance_col;

void main () {
	float s = sin(instance_rot);
	float c = cos(instance_rot);
	mat3 rot_mat = mat3( // Column major for some insane reason!
		 c,  s,  0,
		-s,  c,  0,
		 0,  0,  1
	);
	
	v.world_pos    = rot_mat * mesh_pos + instance_pos;
	v.world_normal = rot_mat * mesh_normal;
	
	v.uv           = mesh_uv;
	v.col          = instance_col;
	
	gl_Position = view.world2clip * vec4(v.world_pos, 1.0);
}

#endif

#ifdef _FRAGMENT
	uniform sampler2D tex;
	
	out vec4 frag_col;
	void main () {
		vec4 col = texture(tex, v.uv);
		
		col.rgb *= simple_lighting(v.world_pos, v.world_normal);
		col.rgb = apply_fog(col.rgb, v.world_pos);
		
		frag_col = col * vec4(v.col, 1.0);
	}
#endif