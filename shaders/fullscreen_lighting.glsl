#version 430
#include "common.glsl"
#include "fullscreen_triangle.glsl"

#ifdef _FRAGMENT
	#define GBUF_IN 1
	#include "gbuf.glsl"
	#define PBR_RENDER 1
	#include "pbr.glsl"
	
#if SHADOWMAP
	#include "shadowmap.glsl"
#endif
	
	out vec4 frag_col;

	void debug_window (sampler2D tex, float lod) {
		vec2 tex_size = vec2(textureSize(tex, 0));
		float tex_aspect = tex_size.x / tex_size.y;
		
		vec2 dbg_sz = vec2(0.4);
		dbg_sz.x *= tex_aspect / view.aspect_ratio;
		vec2 dbg_uv = (v.uv - (vec2(1.0) - dbg_sz)) / dbg_sz;
		if (dbg_uv.x > 0.0 && dbg_uv.y > 0.0) {
			frag_col = vec4(textureLod(tex, dbg_uv, lod).rgb, 1.0);
		}
	}
	void debug_window_shadow (float cascade) {
	#if SHADOWMAP
		vec2 tex_size = vec2(textureSize(shadowmap, 0).xy);
		float tex_aspect = tex_size.x / tex_size.y;
		
		vec2 dbg_sz = vec2(0.4);
		dbg_sz.x *= tex_aspect / view.aspect_ratio;
		vec2 dbg_uv = (v.uv - (vec2(1.0) - dbg_sz)) / dbg_sz;
		if (dbg_uv.x > 0.0 && dbg_uv.y > 0.0) {
			frag_col = vec4(textureLod(shadowmap, vec3(dbg_uv, cascade), 0.0).rgb, 1.0);
		}
	#endif
	}
	
	uniform float visualize_env_lod = 0.0;
	vec3 draw_skybox (in GbufResult g) {
		//vec3 col = readCubemap(night_sky, g.view_dir).rgb;
		//vec3 col = readCubemapLod(pbr_env_map, g.view_dir, round(visualize_env_lod)).rgb * lighting.inv_exposure;
		vec3 col = procedural_sky(view.cam_pos, g.view_dir);
		return col;
	}
	
	void main () {
		GbufResult g;
		bool valid = decode_gbuf(g);
		vec3 col;
		
		if (valid) {
			//g.albedo = vec3(1);
			//g.roughness *= 0.4;
			//g.metallic = 1.0;
			
			col = pbr_IBL(g);
			col += g.emissive;
			
		#if SHADOWMAP
			float shadow = sun_shadowmap(g.pos_world, g.normal_world);
		#else
			float shadow = 1.0;
		#endif 
			if (shadow >= 0.0f) {
				vec3 sun_light = atmos_scattering(lighting.sun_col);
				sun_light *= sun_strength() * 4.0;
				col += shadow * pbr_analytical_light(g, sun_light, dir2sun());
			}
			
			col = apply_fog(col, g.pos_world);
		}
		else {
			col = draw_skybox(g);
		}
		
		frag_col = vec4(col * lighting.exposure, 1.0); // exposure corrected
		
		//debug_window_shadow(0.0);
		//debug_window(gbuf_depth, 0.0);
		//debug_window(gbuf_norm, 0.0);
		//debug_window(gbuf_pbr, 0.0);
		//debug_window(pbr_brdf_LUT, 0.0);
	}
#endif
