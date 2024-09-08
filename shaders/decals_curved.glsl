#version 430
#include "common.glsl"
#include "curved_decal_util.glsl"

#ifdef _FRAGMENT
	
	uniform sampler2D road_wear;
	
	vec4 textureAspectCorrected (sampler2D tex, vec2 uv) {
		vec2 sz = vec2(textureSize(tex, 0));
		uv.y *= sz.x / sz.y;
		return texture(tex, uv);
	}
	
	GBUF_OUT
	void main () {
		vec2 uv; float fade; vec3 pos_world;
		if (!curved_decal(uv, fade, pos_world))
			discard;
		
		vec3 mark = textureAspectCorrected(road_wear, uv).rgb;
		
	#if 1
		vec3  col = mix(0.04, -0.01, mark.r).xxx - mark.ggg*3.0;
		float col_alpha = fade * mix(mark.r, mark.b, 0.20) * 0.6;
		
		float rough = 0.7 - (mark.g * 1.0 + mark.r * 0.25);
		float rough_alpha = mark.b * 0.7;
	#else
		vec3  col = mix(0.05, 0.15, mark.r).xxx - mark.ggg*3.0;
		float col_alpha = fade * mix(mark.r, mark.b, 0.5) * 0.4;
		
		float rough = 0.7 - (mark.g * 1.0 + mark.r * 0.25);
		float rough_alpha = mark.b * 0.7;
	#endif
		
		frag_col   = vec4(col, col_alpha);
		frag_emiss = vec4(0,0,0,0);
		frag_norm  = vec4(0,0,0,0);
		frag_pbr   = vec4(rough,0,0, rough_alpha);
	}
#endif
