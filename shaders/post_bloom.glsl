#version 430
#include "common.glsl"
#include "fullscreen_triangle.glsl"

#ifdef _FRAGMENT
	uniform vec2  texel_sz;
#if PASS <= 1
	uniform sampler2D input;
	uniform float input_mip;
	
#if PASS == 0
	uniform float prefilter_clamp;
	uniform float prefilter_threshold;
	uniform float prefilter_knee;
	
	vec3 bloom_threshold (vec3 col) {
		float x = max(max(col.r, col.g), col.b);
		float t = prefilter_threshold;
		float k = prefilter_knee;
		
		x = min(x, prefilter_clamp);
		
		float a = clamp(x + k - t, 0.0, 2.0 * k);
		float val = max(0.25 * a*a / k, x - t);
		
		col *= val / (x + 0.00001); // works as long as t >= k
		return col;
	}
#endif
	
	vec3 sampl (float offs_x, float offs_y) {
		vec2 uv = v.uv + texel_sz * vec2(offs_x, offs_y);
		return textureLod(input, uv, input_mip).rgb;
	}
	//vec3 bilinear () {
	//	return sampl(0,0);
	//}
	//vec3 box4 () {
	//	vec3 col;
	//	col  = sampl(-0.5,-0.5) + sampl(+0.5,-0.5);
	//	col += sampl(-0.5,+0.5) + sampl(+0.5,+0.5);
	//	
	//	col *= 1.0 / 4.0;
	//	return col;
	//}
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
		
		vec3 col;
		col  = (b+c+d+e) * 0.5  ; // 4 samples weighted 0.5 in total
		col += (f+h+k+m) * 0.125; // 4 samples weighted 0.125 in total
		col += (g+i+j+l) * 0.25 ; // 4 samples weighted 0.125 in total, overlapping 2x
		col +=        a  * 0.125; // 1 sample weighted 0.125, overlapping 4x
		return col * 0.25;
	}
	
	out vec4 frag_col;
	void main () {
		vec3 col = filter_13tap();
	#if PASS == 0
		col = bloom_threshold(col);
	#endif
		
		frag_col = vec4(col, 1.0);
	}
#else
	uniform sampler2D blur_tex;
	uniform sampler2D add_tex;
	uniform float blur_mip;
	uniform float add_mip;
	
	uniform float filter_radius = 1.0;
	
	vec3 sampl (sampler2D tex, float mip, float offs_x, float offs_y) {
		vec2 uv = v.uv + texel_sz * vec2(offs_x, offs_y);
		return textureLod(tex, uv, mip).rgb;
	}
	vec3 filter_tent (sampler2D tex, float mip) {
		const float r = filter_radius;
		
		vec3 col;
		col  = sampl(tex,mip, -r, -r);
		col += sampl(tex,mip,  0, -r) * 2.0;
		col += sampl(tex,mip, +r, -r);
		col += sampl(tex,mip, -r,  0) * 2.0;
		col += sampl(tex,mip,  0,  0) * 4.0;
		col += sampl(tex,mip, +r,  0) * 2.0;
		col += sampl(tex,mip, -r, +r);
		col += sampl(tex,mip,  0, +r) * 2.0;
		col += sampl(tex,mip, +r, +r);
		return col * (1.0 / 16.0);
	}
	
	out vec4 frag_col;
	void main () {
		vec3 blur = filter_tent(blur_tex, blur_mip);
		vec3 add  = texelFetch(add_tex, ivec2(gl_FragCoord.xy), int(add_mip)).rgb;
		
		vec3 col = blur + add;
		frag_col = vec4(col, 1.0);
	}
#endif
#endif
