#version 430
#include "common.glsl"
#include "fullscreen_triangle.glsl"

#ifdef _FRAGMENT
	uniform sampler2D gbuf_depth;
	uniform sampler2D gbuf_col;
	uniform sampler2D gbuf_norm;
	
	out vec4 frag_col;
	void main () {
		float depth = texture(gbuf_depth, v.uv).r;
		vec3 col    = texture(gbuf_col, v.uv).rgb;
		vec3 normal = texture(gbuf_norm, v.uv).rgb;
		
		float len = length(normal);
		if (len > 0.001) {
			normal /= len; // normalize
			
			
			vec3 pos = depth_to_pos_world(depth, v.uv);
			
			col *= simple_lighting(pos, normal);
			//col = apply_fog(col, pos);
			
			//col = overlay_grid(col, pos);
		}
		
		frag_col = vec4(col, 1.0);
		//frag_col = vec4(normal, 1.0);
		//frag_col = vec4(depth,depth,depth, 1.0);
	}
#endif
