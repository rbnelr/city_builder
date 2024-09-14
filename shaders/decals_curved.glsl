#version 430
#include "common.glsl"
#include "curved_decal_util.glsl"

#ifdef _FRAGMENT
	
	vec4 textureAspectCorrected (vec2 uv) {
		vec2 sz = vec2(textureSize(bindless_tex(g.tex, 0), 0));
		uv.y *= sz.x / sz.y;
		
		return texture(bindless_tex(g.tex, 0), flip_y(uv));
	}
	
	GBUF_OUT
	void main () {
		GBUF_HANDLE_WIREFRAME
		
		vec2 uv; float fade; vec3 pos_world;
		if (!curved_decal(uv, fade, pos_world))
			discard;
		
		vec3 mark = textureAspectCorrected(uv).rgb;
		
	#if 0
		vec3  col = mix(0.04, -0.01, mark.r).xxx - mark.ggg*3.0;
		float col_alpha = fade * mix(mark.r, mark.b, 0.20) * 0.7;
		
		float rough = 0.7 - (mark.r * 0.2 + mark.g * 0.6);
		float rough_alpha = clamp((mark.r + mark.g) * 1.0, 0.0, 0.9);
	#else
		vec3  col = mix(0.05, 0.12, mark.r).xxx - mark.ggg*3.0;
		float col_alpha = fade * mix(mark.r, mark.b, 0.5) * 0.3;
		
		float rough = 0.7 - (mark.r * 0.2 + mark.g * 0.6);
		float rough_alpha = clamp((mark.r + mark.g) * 1.0, 0.0, 0.7);
	#endif
		
		frag_col   = vec4(col, col_alpha);
		frag_emiss = vec4(0,0,0,0);
		frag_norm  = vec4(0,0,1, rough_alpha*0.3);
		frag_pbr   = vec4(rough,0,0, rough_alpha);
	}
#endif
