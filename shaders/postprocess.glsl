#version 430
#include "common.glsl"
#include "fullscreen_triangle.glsl"

#ifdef _FRAGMENT
	uniform sampler2D fbo_col;
	
	// https://64.github.io/tonemapping/
	float luminance (vec3 v) {
		return dot(v, vec3(0.2126, 0.7152, 0.0722));
	}
	vec3 change_luminance (vec3 c_in, float l_out) {
		float l_in = luminance(c_in);
		return c_in * (l_out / l_in);
	}
	vec3 tonemap_reinhard2 (vec3 v) {
		float max_white_l = 1.0;
		
		float l_old = luminance(v);
		float numerator = l_old * (1.0 + (l_old / (max_white_l * max_white_l)));
		float l_new = numerator / (1.0 + l_old);
		return change_luminance(v, l_new);
	}
	vec3 reinhard_jodie (vec3 v) {
		float l = luminance(v);
		vec3 tv = v / (1.0f + v);
		return mix(v / (1.0f + l), tv, tv);
	}

	//
	vec3 tonemap_reinhard (vec3 c) {
		return c / (1.0 + c);
	}
	
	vec3 tonemap1 (vec3 c) {
		return c / exp(1.0 - c);
	}
	
	vec3 tonemap_ACESFilmic (vec3 hdr) {
		const float a = 2.51;
		const float b = 0.03;
		const float c = 2.43;
		const float d = 0.59;
		const float e = 0.14;

		return (hdr * (a * hdr + b)) / 
		       (hdr * (c * hdr + d) + e);
	}

	vec3 tonemap_uncharted (vec3 hdr) {
		const float a = 0.22;
		const float b = 0.30;
		const float c = 0.10;
		const float d = 0.20;
		const float e = 0.01;
		const float f = 0.30;

		return ((hdr * (a * hdr + b * c) + d * e) /
		        (hdr * (a * hdr + b) + d * f)) - e / f;
	}
	
	uniform float exposure = 1.0;
	
	out vec4 frag_col;
	void main () {
		vec3 col = texture(fbo_col, v.uv).rgb;
		
		col *= exposure;
		
		col = tonemap_ACESFilmic(col);
		
		frag_col = vec4(col, 1.0);
	}
#endif
