#version 430
#include "common.glsl"
#include "gbuf.glsl"

struct Vertex {
	vec3 pos;
	vec2 uv;
	vec3 normal;
};
VS2FS

#ifdef _VERTEX
	layout(location = 0) in vec2  attr_pos;
	
	//uniform sampler2D heightmap;
	
	uniform vec2  offset;
	uniform float quad_size;
	uniform vec2  inv_max_size;
	
	uniform vec2  lod_bound0;
	uniform vec2  lod_bound1;
	
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
		vec2 uv = pos * inv_max_size;
		//return textureLod(heightmap, uv, 0.0).r * 1200.0 - 100.0;
		return 0.0;
	}
	
	void main () {
		vec3 pos = vec3(quad_size * attr_pos + offset, 0.0);
		
		fix_lod_seam(pos.xy);
		
		pos.z = sample_heightmap(pos.xy);
		
		{ // numerical derivative of heightmap for normals
			float eps = max(quad_size, 2.0);
			vec3 b = vec3(pos.x + eps, pos.y, 0.0);
			vec3 c = vec3(pos.x, pos.y + eps, 0.0);
			
			b.z = sample_heightmap(b.xy);
			c.z = sample_heightmap(c.xy);
			
			v.normal = normalize(cross(b - pos, c - pos));
		}
		
		gl_Position = view.world2clip * vec4(pos, 1.0);
		v.pos = pos;
		v.uv  = pos.xy * inv_max_size;
	}
#endif
#ifdef _FRAGMENT
	uniform sampler2D terrain_diffuse;
	
	GBUF_OUT
	void main () {
		frag_col = vec4(texture(terrain_diffuse, v.pos.xy / 32.0).rgb, 1.0);
		frag_norm = vec4(v.normal, 1.0);
	}
#endif
