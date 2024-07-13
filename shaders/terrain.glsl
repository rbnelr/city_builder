#version 430
#include "common.glsl"
#include "gbuf.glsl"

VS2FS Vertex {
	vec3 pos;
	vec2 uv;
	vec3 normal;
	vec3 tint;
} v;

#ifdef _VERTEX
	layout(location = 0) in vec2  attr_pos;
	
	uniform sampler2D heightmap;
	uniform sampler2D heightmap_outer;
	
	uniform vec2  offset;
	uniform float quad_size;
	
	uniform vec2  inv_map_size;
	uniform vec2  inv_outer_size;
	
	uniform float z_min;
	uniform float z_range;
	
	uniform vec2  lod_bound0;
	uniform vec2  lod_bound1;
	
	uniform vec4 dbg_tint;
	
	// TODO: could be done on cpu side by providing a mesh variant for
	// center, edge and corner lod transitions and having the cpu code select them
	// they would have to be rotated with a 2x2 matrix
	void fix_lod_seam (inout vec2 pos) {
		//vs_dbg = vec3(0);
		
		vec2 dist = min(lod_bound1 - pos, pos - lod_bound0);
		vec2 t = clamp(1.0 - dist / quad_size, vec2(0), vec2(1));
		
		float tm = max(t.x, t.y);
		
		if (tm > 0.0) {
			
			vec2 shift = fract(attr_pos * 0.5) * 2.0 * quad_size;
			
			if (tm == t.x) pos.y -= shift.y * t.x;
			else           pos.x -= shift.x * t.y;
			
			//vs_dbg = vec3(t, 0.0);
		}
	}
	
	float sample_heightmap (vec2 pos) {
		if (  abs(pos.x * inv_map_size.x) <= 0.5 &&
			  abs(pos.y * inv_map_size.y) <= 0.5  ) {
			vec2 uv = pos * inv_map_size + 0.5;
			//return 0.0;
			return textureLod(heightmap, uv, 0.0).r * z_range + z_min;
		}
		else {
			vec2 uv = pos * inv_outer_size + 0.5;
			//return 0.0;
			return textureLod(heightmap_outer, uv, 0.0).r * z_range + z_min;
		}
	}
	
	void main () {
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
		
		if (dbg_tint.a > 0.0)
			v.tint = dbg_tint.rgb;
		
		gl_Position = view.world2clip * vec4(pos, 1.0);
		v.pos = pos;
		v.uv  = pos.xy * inv_map_size;
	}
#endif
#ifdef _FRAGMENT
	uniform sampler2D terrain_diffuse;
	uniform vec4 pbr = vec4(0.95, 0,0,1);
	
	GBUF_OUT
	void main () {
		vec3 col = v.tint * texture(terrain_diffuse, v.pos.xy / 32.0).rgb;
	
		//col = overlay_grid(col, v.pos);
		//col = overlay_countour_lines(col, v.pos);
		
		frag_col = vec4(col, 1.0);
		frag_norm = vec4(v.normal, 1.0);
		frag_pbr = pbr;
	}
#endif
