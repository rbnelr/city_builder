#version 430
#include "common.glsl"
#include "curved_decal_util.glsl"

#ifdef _FRAGMENT
	
	uniform int pattern_base_texid;
	uniform int overlay_base_texid;
	
	float antialised (float value, float thres, float antialis_px) {
		float df = fwidth(value) * antialis_px;
		float t = (value - thres + df*0.5) / df;
		return clamp(t, 0.0, 1.0);
	}
	
	vec4 texture_fitted (vec2 uv, int tex) {
		const float tex_aspect = 2.0;
		
		{
			// weird math that causes texture to get stretched or squashed in the middle while tip and tail are kept constant aspect
			// % of texture that are considered tip/tail
			const float q = 0.333;
			float q2 = q * 2.0;
			float l = g.uv_remap.w / tex_aspect;
			float x = uv.y / tex_aspect;
			
			if (l < 1.0) {
				uv.y = x / l;
			}
			else {
				float a = x;
				float b = x - (l - 1.0);
				float c = ( x*(1.0 - q2) + q*(l - 1.0) ) / (l - q2);
				
				uv.y = clamp(max(min(a, c), b), 0.0, 1.0);
			}
		}
		
		float greyscale = bindless_tex_scaled(overlay_base_texid + tex, 0, flip_y(uv)).r;
		
		vec4 col = g.col;
		col.a *= greyscale;
		
		//col = vec4(fract(uv.yy), 0, 1);
		
		return col;
	}
	
	vec4 pattern (vec2 uv, vec3 pos_world, int tex) {
		// -1 => pattern_base_texid, -2 => pattern_base_texid+1 etc.
		float pat = bindless_tex_scaled(pattern_base_texid + (-1-tex), 0, flip_y(pos_world.xy)).r;
		
		vec4 col_inner   = g.col * vec4(1,1,1, 0.9) * pat;
		vec4 col_outline = g.col * vec4(0.1,0.1,0.1, 1.0);
		
		vec4 col;
		{ // draw outline
			vec2 dist = min(uv - g.uv_remap.xy, g.uv_remap.zw - uv); // dist to edge in meters
			float edge = min(dist.x, dist.y);
			
			float edge0 = antialised(edge, 0.01, 2.0);
			float edge1 = antialised(edge, 0.05, 2.0);
			
			col = mix(col_outline, col_inner, edge1);
			col.a *= edge0;
		}
		
		return col;
	}
	
	GBUF_OUT
	void main () {
		GBUF_HANDLE_WIREFRAME
		
		vec2 uv; float fade; vec3 pos_world;
		if (!curved_decal(uv, fade, pos_world))
			discard;
		
		vec4 col;
		if (g.tex >= 0) {
			col = texture_fitted(uv, g.tex);
		}
		else {
			col = pattern(uv, pos_world, g.tex);
		}
		
		col.a *= fade;
	#if 0
		// sets the albedo
		frag_col   = col;
		frag_emiss = vec4(0,0,0, col.a);
		frag_norm  = vec4(0,0,1, col.a);
		frag_pbr   = vec4(1,0,0, col.a);
	#else
		// use emissive (not multiplied by exposure factor like usual, so it stays constant brightness!)
		// albedo, normal and pbr factors still get blended with constants and alpha,
		// so the emissive acts more like an albedo that is lit by a constant 1 brightness light!
		frag_col   = vec4(0,0,0, col.a); // black albedo
		frag_emiss = col;
		frag_norm  = vec4(0,0,1, col.a); // up facing normal
		frag_pbr   = vec4(1,0,0, col.a); // 100% roughness, 0% metallic
	#endif
	}
#endif
