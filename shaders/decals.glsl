#version 430
#include "common.glsl"
#include "gbuf.glsl"

struct Vertex {
	vec3 pos;
	vec3 norm;
	vec2 uv;
	vec4 col;
	float tex_id;
};
VS2FS

#ifdef _VERTEX
	layout(location = 0) in vec3  pos;
	layout(location = 1) in vec3  norm;
	layout(location = 2) in vec2  uv;
	layout(location = 3) in vec4  col;
	layout(location = 4) in float tex_id;
	
	void main () {
		gl_Position = view.world2clip * vec4(pos, 1.0);
		v.pos  = pos;
		v.norm  = norm;
		v.uv  = uv;
		v.col = col;
		v.tex_id = tex_id;
	}
#endif
#ifdef _FRAGMENT
	uniform sampler2DArray turn_arrows;
	uniform sampler2D cracks;
	
	GBUF_OUT
	void main () {
		vec4 col = texture(turn_arrows, vec3(v.uv, v.tex_id)) * v.col;
		//col.a = 1.0;
		
		float alpha = col.a;
		alpha *= texture(cracks, v.pos.xy * 0.2).r;
		
		alpha = clamp(map(alpha - 0.12, 0.0, 0.10), 0.0, 1.0);
		
		col.a = alpha * 0.8;
		col.rgb *= 0.7;
		
		frag_col = col;
		frag_norm = vec4(v.norm, 0.7 * col.a);
	}
#endif
