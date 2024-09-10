#version 430
#include "common.glsl"

VS2FS Vertex {
	vec3 pos;
	vec2 uv;
	vec3 normal;
	vec3 tint;
} v;

VS2FS vec3 wireframe_barycentric;

#ifdef _VERTEX
	layout(location = 0) in vec2  attr_pos;
	layout(location = 1) in vec3  offset_scale;
	layout(location = 2) in vec4  lod_bounds;
	
	uniform sampler2D heightmap;
	uniform sampler2D heightmap_outer;
	
	uniform vec2  inv_map_size;
	uniform vec2  inv_outer_size;
	
	uniform float height_min;
	uniform float height_range;
	
	// TODO: could be done on cpu side by providing a mesh variant for
	// center, edge and corner lod transitions and having the cpu code select them
	// they would have to be rotated with a 2x2 matrix
	void fix_lod_seam (inout vec2 pos) {
		vec2 lod_bound0 = lod_bounds.xy;
		vec2 lod_bound1 = lod_bounds.zw;
		float quad_size = offset_scale.z;
		
		vec2 dist = min(lod_bound1 - pos, pos - lod_bound0);
		vec2 t = clamp(1.0 - dist / quad_size, vec2(0), vec2(1));
		
		float tm = max(t.x, t.y);
		
		if (tm > 0.0) {
			vec2 shift = fract(attr_pos * 0.5) * 2.0 * quad_size;
			
			if (tm == t.x) pos.y -= shift.y * t.x;
			else           pos.x -= shift.x * t.y;
		}
	}
	
	float sample_heightmap (vec2 pos) {
		//return 0.0;
		
		//float quad_size = offset_scale.z;
		float tex_lod = 0.0;
		
		if (  abs(pos.x * inv_map_size.x) <= 0.5 &&
			  abs(pos.y * inv_map_size.y) <= 0.5  ) {
			vec2 uv = pos * inv_map_size + 0.5;
			//return 0.0;
			
			// Causes seams
			//float texel_dens = textureSize(heightmap, 0).x * inv_map_size.x;
			//float tex_lod = log2(quad_size * texel_dens);
			return textureLod(heightmap, uv, tex_lod).r * height_range + height_min;
		}
		else {
			vec2 uv = pos * inv_outer_size + 0.5;
			//return 0.0;
			
			//float texel_dens = textureSize(heightmap_outer, 0).x * inv_outer_size.x;
			//float tex_lod = log2(quad_size * texel_dens);
			return textureLod(heightmap_outer, uv, tex_lod).r * height_range + height_min;
		}
	}
	
	void main () {
		vec2 offset = offset_scale.xy;
		float quad_size = offset_scale.z;
		
		vec3 pos = vec3(quad_size * attr_pos + offset, 0.0);
		
		fix_lod_seam(pos.xy);
		
		pos.z = sample_heightmap(pos.xy);
		
		{ // numerical derivative of heightmap for normals
			float eps = clamp(quad_size, 2.0, 16.0);
			vec3 b = vec3(pos.x + eps, pos.y, 0.0);
			vec3 c = vec3(pos.x, pos.y + eps, 0.0);
			
			b.z = sample_heightmap(b.xy);
			c.z = sample_heightmap(c.xy);
			
			v.normal = normalize(cross(b - pos, c - pos));
		}
		
		v.tint = vec3(1.0);
		
		{
			const float a=0.005, b=0.05;
			float cliff = 1.0 - v.normal.z;
			cliff = clamp((cliff-a) / (b-a), 0.0, 1.0)*0.8;
			
			v.tint = mix(v.tint, vec3(0.02,0.01,0.005), cliff);
		}
		{
			const float a=0.0, b=0.003;
			float cliff = 1.0 - v.normal.z;
			cliff = clamp((cliff-a) / (b-a), 0.0, 1.0)*0.8;
			
			v.tint = mix(v.tint, vec3(0.5,0.3,0.5), cliff);
		}
		
		//if (dbg_tint.a > 0.0)
		//	v.tint = dbg_tint.rgb;
		
		gl_Position = view.world2clip * vec4(pos, 1.0);
		v.pos = pos;
		v.uv  = pos.xy * inv_map_size;
	}
#endif
#ifdef _FRAGMENT
	#include "gbuf.glsl"
	
	uniform sampler2D terrain_diffuse;
	uniform vec4 pbr = vec4(0.95,0.0, 0,1);
	
	uniform bool contour_lines = false;
	
	GBUF_OUT
	void main () {
		GBUF_HANDLE_WIREFRAME
		
		vec2 uv = v.pos.xy / 32.0;
		uv.y = 1.0 - uv.y; // all textures are flipped!
		vec3 col = v.tint * texture(terrain_diffuse, v.pos.xy / 32.0).rgb;
	
		//col = overlay_grid(col, v.pos);
		if (contour_lines)
			col = overlay_countour_lines(col, v.pos);
		
		frag_col   = vec4(col, 1.0);
		frag_emiss = vec4(0,0,0,1);
		frag_norm  = vec4(v.normal, 1.0);
		frag_pbr   = pbr;
	}
#endif
