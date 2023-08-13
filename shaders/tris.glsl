#version 430
#include "common.glsl"

struct Vertex {
	vec2 uv;
	vec4 col;
};
VS2FS

#ifdef _VERTEX
	layout(location = 0) in vec3  pos;
	layout(location = 1) in vec2  uv;
	layout(location = 2) in vec4  col;
	
	void main () {
		gl_Position = view.world2clip * vec4(pos, 1.0);
		v.uv  = uv;
		v.col = col;
	}
#endif
#ifdef _FRAGMENT
	//uniform sampler2D tex;
	
	out vec4 frag_col;
	void main () {
		//frag_col = texture(tex, v.uv) * v.col;
		frag_col = v.col;
	}
#endif
