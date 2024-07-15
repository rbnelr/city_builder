#version 430
#include "common.glsl"
#include "fullscreen_triangle.glsl"

#define SHADOWMAP 1

#ifdef _FRAGMENT
	#define GBUF_IN 1
	#include "gbuf.glsl"
	#define PBR_RENDER 1
	#include "pbr.glsl"
	
#if SHADOWMAP
	#include "shadowmap.glsl"
#endif
	
	out vec4 frag_col;
	
	void debug_window (sampler2D tex) {
		vec2 tex_size = vec2(textureSize(tex, 0));
		float tex_aspect = tex_size.x / tex_size.y;
		
		vec2 dbg_sz = vec2(0.4);
		dbg_sz.x *= tex_aspect / view.aspect_ratio;
		vec2 dbg_uv = (v.uv - (vec2(1.0) - dbg_sz)) / dbg_sz;
		if (dbg_uv.x > 0.0 && dbg_uv.y > 0.0) {
			frag_col = vec4(texture(tex, dbg_uv).rgb, 1.0);
		}
	}
	void debug_window_shadow (sampler2DArray tex, float cascade) {
		vec2 tex_size = vec2(textureSize(tex, 0).xy);
		float tex_aspect = tex_size.x / tex_size.y;
		
		vec2 dbg_sz = vec2(0.4);
		dbg_sz.x *= tex_aspect / view.aspect_ratio;
		vec2 dbg_uv = (v.uv - (vec2(1.0) - dbg_sz)) / dbg_sz;
		if (dbg_uv.x > 0.0 && dbg_uv.y > 0.0) {
			frag_col = vec4(texture(tex, vec3(dbg_uv, cascade)).rgb, 1.0);
		}
	}
	
	vec3 draw_skybox (in GbufResult g) {
	#if 1
		vec3 col = procedural_sky(view.cam_pos, g.view_dir);
		col = apply_fog(col, view.cam_pos + g.view_dir * 1000000.0);
		return col;
	#else
		return readCubemap(pbr_spec_env, g.view_dir).rgb;
	#endif
	}
	
	void main () {
		GbufResult g;
		bool valid = decode_gbuf(g);
		vec3 col = vec3(0);
		//col = vec3(.4);
		
		if (valid) {
		#if SHADOWMAP
			float shadow = sun_shadowmap(g.pos_world, g.normal_world);
		#else
			float shadow = 1.0;
		#endif
			
			//g.albedo = vec3(1);
			//g.roughness *= 1.5;
			//g.metallic = 0.0;
			
			{
				//if (gl_FragCoord.x > 950) {
				//if (bool(1)) {
					//col = pbr_reference_env_light(g);
					//col = pbr_approx_env_light(g);
				//}
				//else {
					col = pbr_approx_env_light(g);
					//col = pbr_approx_env_light_test(g);
				//}
				
				vec3 sun_light = lighting.sun_col - atmos_scattering();
				sun_light *= sun_strength() * 2.0;
				//col += pbr_directional_light(g, sun_light, -lighting.sun_dir);
			}
			
			col = apply_fog(col, g.pos_world);
			//col = vec3(g.roughness);
			
			//col = overlay_grid(col, g.pos_world);
			//col = overlay_countour_lines(col, g.pos_world);
		}
		else {
			col = draw_skybox(g);
		}
		
		frag_col = vec4(col, 1.0);
		//frag_col = vec4(normal, 1.0);
		//frag_col = vec4(depth,depth,depth, 1.0);
		
		//debug_window_shadow(shadowmap, 1.0);
		//debug_window(gbuf_depth);
		//debug_window(gbuf_norm);
		//debug_window(gbuf_pbr);
		//debug_window(pbr_brdf_LUT);
	}
#endif
