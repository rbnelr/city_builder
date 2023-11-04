#version 430
#include "common.glsl"
#include "fullscreen_triangle.glsl"

#ifdef _FRAGMENT
	uniform sampler2D gbuf_depth;
	uniform sampler2D gbuf_col;
	uniform sampler2D gbuf_norm;
	
	uniform sampler2DShadow shadowmap;
	uniform mat4 shadowmap_mat;
	uniform vec3 shadowmap_dir; // light dir of sun
	
	out vec4 frag_col;
	
	float sun_shadowmap (vec3 pos_world, vec3 normal) {
		// should depend on shadowmap resolution!
		float bias = clamp(0.0010 * (1.0 - dot(normal, -shadowmap_dir)), 0.0, 0.0010);
		//float bias = 0.0005;
		//frag_col = vec4(bias.xxx * 20.0, 1.0);
		
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
		// Let gpu do comparisons compare > shadow texel for us and then do bilinear filtering
		float shadow_fac = texture(shadowmap, vec3(shadow_uv, compare)).r;
		return shadow_fac;
	}
	
	void main () {
		float depth = texture(gbuf_depth, v.uv).r;
		vec3 col    = texture(gbuf_col, v.uv).rgb;
		vec3 normal = texture(gbuf_norm, v.uv).rgb;
		
		float len = length(normal);
		if (len > 0.001) {
			normal /= len; // normalize
			vec3 pos = depth_to_pos_world(depth, v.uv);
			
			float shadow = sun_shadowmap(pos, normal);
			
			col *= sun_lighting(normal, shadow);
			//col = apply_fog(col, pos);
			
			//col = overlay_grid(col, pos);
		}
		
		frag_col = vec4(col, 1.0);
		//frag_col = vec4(normal, 1.0);
		//frag_col = vec4(depth,depth,depth, 1.0);
	}
#endif
