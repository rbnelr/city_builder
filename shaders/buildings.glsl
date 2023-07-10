#version 430
#include "common.glsl"

struct Vertex {
	vec3 world_pos;
	vec3 world_normal;
	vec2 uv;
};
VS2FS

#ifdef _VERTEX
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
