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
	
	uniform float _visualize_roughness;

	vec3 draw_skybox (in GbufResult g) {
	#if 0
		vec3 col = procedural_sky(view.cam_pos, g.view_dir);
		return col;
	#else
		return pbr_sample_specular_env_map(g.view_dir, _visualize_roughness).rgb;
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
			//g.roughness *= 0.4;
			//g.metallic = 1.0;
			
			{
				//if (gl_FragCoord.x > 950) {
				//if (bool(1)) {
				//	col = pbr_reference_env_light(g);
					//col = pbr_approx_env_light(g);
				//} else {
					col = pbr_IBL(g);
					//col = pbr_IBL_test(g);
				//}
				
				//float shadow = sun_shadowmap(g.pos_world, g.normal_world);
				//if (shadow >= 0.0f) {
					vec3 sun_light = atmos_scattering(lighting.sun_col);
					sun_light *= sun_strength() * 3.0;
					col += /*shadow * */pbr_analytical_light(g, sun_light, -lighting.sun_dir);
				//}
				//col = shadow.xxx;
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
		
		//debug_window_shadow(shadowmap, 0.0);
		//debug_window(gbuf_depth);
		//debug_window(gbuf_norm);
		//debug_window(gbuf_pbr);
		//debug_window(pbr_brdf_LUT);
	}
#endif
