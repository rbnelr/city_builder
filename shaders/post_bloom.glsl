#version 430
#include "common.glsl"
#include "fullscreen_triangle.glsl"

#ifdef _FRAGMENT
	uniform sampler2D input;
	
	out vec4 frag_col;
	void main () {
		vec3 col = textureLod(input, v.uv, 0.0).rgb;
		
		frag_col = vec4(col, 1.0);
	}
#endif
