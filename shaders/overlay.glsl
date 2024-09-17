#version 430
#include "common.glsl"
#include "curved_decal_util.glsl"

#ifdef _FRAGMENT
	
	uniform int base_texid;
	
	float antialised (float value, float thres, float antialis_px) {
		float df = fwidth(value) * antialis_px;
		float t = (value - thres + df*0.5) / df;
		return clamp(t, 0.0, 1.0);
	}
	
	vec4 pattern (vec2 uv, vec3 pos_world, int texid) {
		const int TEXTURE_THICK_ARROW = 2;
		
		if (v.tex >= TEXTURE_THICK_ARROW) {
			//uv.y /= v.uv_len;
			
			float tex = texture(bindless_tex(base_texid + texid, 0), flip_y(uv)).r;
			return vec4(1,1,1, tex);
		}
		else {
			float tex = bindless_tex_scaled(base_texid + texid, 0, flip_y(pos_world.xy)).r;
			
			vec4 col_inner   = vec4(1.0,1.0,1.0, 0.9) * tex;
			vec4 col_outline = vec4(0.1,0.1,0.1, 1.0);
			
			vec4 col;
			{ // draw outline
				float distX = min(uv.x, 1.0 - uv.x); // dist to edge in meters
				float distY = min(uv.y, v.uv_len - uv.y); // dist to edge in meters
				float edge = min(distX, distY);
				
				float edge0 = antialised(edge, 0.01, 2.0);
				float edge1 = antialised(edge, 0.05, 2.0);
				
				col = mix(col_outline, col_inner, edge1);
				col.a *= edge0;
			}
			
			return col;
		}
	}
	
	GBUF_OUT
	void main () {
		GBUF_HANDLE_WIREFRAME
		
		vec2 uv; vec4 col; vec3 pos_world;
		if (!curved_decal(uv, col, pos_world))
			discard;
		
		col *= pattern(uv, pos_world, v.tex);
		
		// use emissive (not multiplied by exposure factor like usual, so it stays constant brightness!)
		// albedo, normal and pbr factors still get blended with constants and alpha,
		// so the emissive acts more like an albedo that is lit by a constant 1 brightness light!
		frag_col   = vec4(0,0,0, col.a); // black albedo
		frag_emiss = col;
		frag_norm  = vec4(0,0,1, col.a); // up facing normal
		frag_pbr   = vec4(1,0,0, col.a); // 100% roughness, 0% metallic
	}
#endif
