#version 430
#include "common.glsl"
#include "fullscreen_triangle.glsl"

#ifdef _FRAGMENT
	#define GBUF_IN 1
	#include "gbuf.glsl"

	uniform sampler2D shadowmap;
	uniform sampler2DShadow shadowmap2;
	uniform mat4 shadowmap_mat;
	uniform vec3 shadowmap_dir; // light dir of sun
	uniform float shadowmap_bias_fac = 0.0005;
	uniform float shadowmap_bias_max = 0.004;
	
	out vec4 frag_col;
	
	float sun_shadowmap (vec3 pos_world, vec3 normal) {
		float bias = shadowmap_bias_fac * tan(acos(dot(normal, -shadowmap_dir)));
		bias = clamp(bias, 0.0, shadowmap_bias_max);
		
		vec4 shadow_clip = shadowmap_mat * vec4(pos_world, 1.0);
		vec3 shadow_ndc = shadow_clip.xyz / shadow_clip.w;
		vec2 shadow_uv = shadow_ndc.xy * 0.5 + 0.5;
		
		// TODO: fix this function wihout reverse_depth?
		// ndc is [-1,1] except for with z with reverse depth which is [0,1]
		// NOTE: not to be confused, shadow rendering uses ortho camera which uses a linear depth range unlike normal camera!
		// could use [-1,1] range here
		if (shadow_ndc.z < 0.0)
			return 1.0;
		
		float compare = shadow_ndc.z + bias;
		
		float shadow_fac = 0.0;
		
		// PCF
		vec2 texelSize = 1.0 / textureSize(shadowmap, 0);
		int sum = 0;
		for (int x=-1; x<=1; ++x)
		for (int y=-1; y<=1; ++y) {
			vec2 uv = shadow_uv + vec2(x,y) * texelSize;
			
			float fac = texture(shadowmap2, vec3(uv, compare)).r;
			//float fac = compare > texture(shadowmap, uv).r ? 1.0 : 0.0;
			shadow_fac += fac;
			sum++;
		}
		shadow_fac /= float(sum);
		
		return shadow_fac;
	}
	
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
	
	void main () {
		GbufResult g;
		bool valid = decode_gbuf(g);
		//col = vec3(1);
		
		vec3 col = g.albedo;
		
		if (valid) {
			float shadow = sun_shadowmap(g.pos_world, g.norm_world);
			
			col *= sun_lighting(g.norm_world, shadow);
			//col = apply_fog(col, pos);
			
			//col = overlay_grid(col, pos);
		}
		
		frag_col = vec4(col, 1.0);
		//frag_col = vec4(normal, 1.0);
		//frag_col = vec4(depth,depth,depth, 1.0);
		
		//debug_window(shadowmap);
		//debug_window(gbuf_depth);
		//debug_window(gbuf_norm);
	}
#endif