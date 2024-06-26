#version 430
#include "common.glsl"
#include "fullscreen_triangle.glsl"

#ifdef _FRAGMENT
	#define GBUF_IN 1
	#include "gbuf.glsl"

#define SHADOWMAP 1
#if SHADOWMAP
	uniform sampler2DArray shadowmap;
	uniform sampler2DArrayShadow shadowmap2;
	uniform mat4 shadowmap_mat;
	uniform vec3 shadowmap_dir; // light dir of sun
	uniform float shadowmap_bias_fac = 0.0005;
	uniform float shadowmap_bias_max = 0.004;
	uniform float shadowmap_cascade_factor;
	
	float sun_shadowmap (vec3 pos_world, vec3 normal) {
		vec2 texelSize = 1.0 / textureSize(shadowmap, 0).xy;
		int cascades = int(textureSize(shadowmap, 0).z);
		
		
		float bias = shadowmap_bias_fac * tan(acos(dot(normal, -shadowmap_dir)));
		bias = clamp(bias, 0.0, shadowmap_bias_max);
		
		vec4 shadow_clip = shadowmap_mat * vec4(pos_world, 1.0);
		vec3 shadow_ndc = shadow_clip.xyz / shadow_clip.w;
		// [0,1] -> [-1,+1] because else the cascade logic becomes harder
		shadow_ndc.z = shadow_ndc.z*2.0 - 1.0;
		
		float m = max(max(abs(shadow_ndc.x), abs(shadow_ndc.y)),
			abs(shadow_ndc.z));
		m = log(m) / log(shadowmap_cascade_factor);
		float cascade = max(ceil(m), 0.0);
		
		float scale = pow(shadowmap_cascade_factor, cascade);
		
		if (cascade >= cascades)
			return 1.0;
		
		shadow_ndc /= scale;
		
		vec2 shadow_uv = shadow_ndc.xy * 0.5 + 0.5;
		
		// TODO: fix this function wihout reverse_depth?
		// ndc is [-1,1] except for with z with reverse depth which is [0,1]
		// NOTE: not to be confused, shadow rendering uses ortho camera which uses a linear depth range unlike normal camera!
		// could use [-1,1] range here
		
		shadow_ndc.z = shadow_ndc.z*0.5 + 0.5; // [-1,+1] -> [0,1]
		if (shadow_ndc.z < 0.0)
			return 1.0;
		
		// TODO: need different bias for other cascades?
		float compare = shadow_ndc.z + bias;
		
		float shadow_fac = 0.0;
		
		// PCF
		int sum = 0;
		for (int x=-1; x<=1; ++x)
		for (int y=-1; y<=1; ++y) {
			vec2 uv = shadow_uv + vec2(x,y) * texelSize;
			
			float fac = texture(shadowmap2, vec4(uv, cascade, compare)).r;
			//float fac = compare > texture(shadowmap, vec3(uv, float(cascade))).r ? 1.0 : 0.0;
			shadow_fac += fac;
			sum++;
		}
		shadow_fac /= float(sum);
		
		return shadow_fac;
	}
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
	
	void main () {
		GbufResult g;
		bool valid = decode_gbuf(g);
		vec3 col = g.albedo;
		//col = vec3(.4);
		
		if (valid) {
		#if SHADOWMAP
			float shadow = sun_shadowmap(g.pos_world, g.norm_world);
		#else
			float shadow = 1.0;
		#endif
			
			//col = overlay_grid(col, g.pos_world);
			//col = overlay_countour_lines(col, g.pos_world);
			
			col *= sun_lighting(g.norm_world, shadow);
			col = apply_fog(col, g.pos_world);
		}
		
		frag_col = vec4(col, 1.0);
		//frag_col = vec4(normal, 1.0);
		//frag_col = vec4(depth,depth,depth, 1.0);
		
		//debug_window_shadow(shadowmap, 1.0);
		//debug_window(gbuf_depth);
		//debug_window(gbuf_norm);
	}
#endif
