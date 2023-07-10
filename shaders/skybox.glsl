#version 430
#include "common.glsl"

struct Vertex {
	vec3 pos;
};
VS2FS

#ifdef _VERTEX
	layout(location = 0) in vec3 pos;
	
	void main () {
		vec3 cam = mat3(view.world2cam) * pos; // world space to cam rotation only
		
		// w=0 to draw at far plane (with GL_DEPTH_CLAMP)
		gl_Position = view.cam2clip * vec4(cam, 0.0);
		v.pos = pos;
	}
#endif
#ifdef _FRAGMENT
	out vec4 frag_col;
	void main () {
		vec3 dir = normalize(v.pos); // worldspace dir
		
		vec3 col = get_skybox_light(view.cam_pos, dir);
		col = apply_fog(col, view.cam_pos + dir * 100000.0);
		frag_col = vec4(col, 1.0);
	}
#endif
