#version 430
#include "common.glsl"
#include "fullscreen_triangle.glsl"

#ifdef _FRAGMENT
	uniform sampler2D input;
	uniform float input_mip;
	
	uniform vec2  texel_sz;
	
	vec3 sampl (float offs_x, float offs_y) {
		vec2 uv = v.uv + texel_sz * vec2(offs_x, offs_y);
		return textureLod(input, uv, src_mip).rgb;
	}
	vec3 bilinear () {
		return sampl(0,0);
	}
	vec3 box4 () {
		vec3 col;
		col  = sampl(-0.5,-0.5) + sampl(+0.5,-0.5);
		col += sampl(-0.5,+0.5) + sampl(+0.5,+0.5);
		
		col *= 1.0 / 4.0;
		return col;
	}
	vec3 filter_13tap () {
		vec3 a = sampl(0, 0);
		
		vec3 b = sampl(-0.5,-0.5);
		vec3 c = sampl(+0.5,-0.5);
		vec3 d = sampl(-0.5,+0.5);
		vec3 e = sampl(+0.5,+0.5);
		
		vec3 f = sampl(-1, -1);
		vec3 g = sampl( 0, -1);
		vec3 h = sampl(+1, -1);
		vec3 i = sampl(-1,  0);
		vec3 j = sampl(+1,  0);
		vec3 k = sampl(-1, +1);
		vec3 l = sampl( 0, +1);
		vec3 m = sampl(+1, +1);
		
		vec3 col = (b+c+d+e)*0.5; // 4 samples weighted 0.5 in total
		col += (f+h+k+m) * 0.125; // 4 samples weighted 0.125 in total
		col += (g+i+j+l) * 0.25;  // 4 samples weighted 0.125 in total, overlapping 2x
		col += a * 0.125;         // 1 sample weighted 0.125, overlapping 4x
		return col * 0.25;
	}
	
#if PASS == 1
	out vec4 frag_col;
	void main () {
		vec3 col = filter_13tap();
		
		frag_col = vec4(col, 1.0);
	}
#else
	out vec4 frag_col;
	void main () {
		vec3 col = bilinear();
		
		frag_col = vec4(col, 1.0);
	}
#endif

#endif
