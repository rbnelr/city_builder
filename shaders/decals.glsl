#version 430
#include "common.glsl"

struct Vertex {
	vec2 uv;
	vec4 col;
	float tex_id;
};
VS2FS

#ifdef _VERTEX
	layout(location = 0) in vec3  pos;
	layout(location = 1) in vec2  uv;
	layout(location = 2) in vec4  col;
	layout(location = 3) in float tex_id;
	
	void main () {
		gl_Position = view.world2clip * vec4(pos, 1.0);
		v.uv  = uv;
		v.col = col;
		v.tex_id = tex_id;
	}
#endif
#ifdef _FRAGMENT
	uniform sampler2DArray turn_arrows;
	
	out vec4 frag_col;
	void main () {
		frag_col = texture(turn_arrows, vec3(v.uv, v.tex_id)) * v.col;
	}
#endif
