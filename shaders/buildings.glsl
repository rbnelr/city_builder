#version 430
#include "common.glsl"

struct Vertex {
	vec3 world_pos;
	vec3 world_normal;
	vec2 uv;
};
VS2FS

#ifdef _VERTEX

#if MODE == 0
	layout(location = 0) in vec3  pos;
	layout(location = 1) in vec3  normal;
	layout(location = 2) in vec2  uv;
	
	uniform mat4 model2world;
	
	void main () {
		gl_Position = view.world2clip * model2world * vec4(pos, 1.0);
		v.world_pos    = pos;
		v.world_normal = normal;
		v.uv           = uv;
	}
#else
	layout(location = 0) in vec3  mesh_pos;
	layout(location = 1) in vec3  mesh_normal;
	layout(location = 2) in vec2  mesh_uv;
	layout(location = 3) in vec3  instance_pos;
	layout(location = 4) in float instance_rot;
	
	void main () {
		gl_Position = view.world2clip * vec4(mesh_pos + instance_pos, 1.0);
		v.world_pos    = mesh_pos;
		v.world_normal = mesh_normal;
		v.uv           = mesh_uv;
	}
#endif

#endif

#ifdef _FRAGMENT
	uniform sampler2D tex;
	
	out vec4 frag_col;
	void main () {
		vec4 col = texture(tex, v.uv);
		
		col.rgb *= simple_lighting(v.world_pos, v.world_normal);
		col.rgb = apply_fog(col.rgb, v.world_pos);
		
		frag_col = col;
	}
#endif
