#version 430
#include "common.glsl"
#include "fullscreen_triangle.glsl"

#ifdef _FRAGMENT
	#include "pbr.glsl"
	
	uniform vec2 resolution;
	
	out vec4 frag_col;
	void main () {
		vec2 uv = gl_FragCoord.xy / resolution;
		
		vec2 brdf = integrate_brdf(uv.x, uv.y, 4096);
		
		frag_col = vec4(brdf, 0.0, 1.0);
	}
#endif
