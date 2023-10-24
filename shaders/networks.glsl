#version 430
#include "common.glsl"

struct Vertex {
	vec3 pos;
	vec3 norm;
	float tex_id;
};
VS2FS

#ifdef _VERTEX
	layout(location = 0) in vec3  pos;
	layout(location = 1) in vec3  norm;
	layout(location = 2) in float tex_id;
	
	void main () {
		gl_Position = view.world2clip * vec4(pos, 1.0);
		v.pos  = pos;
		v.norm = norm;
		v.tex_id = tex_id;
	}
#endif
#ifdef _FRAGMENT
	uniform sampler2DArray surfaces_color;
	uniform sampler2DArray surfaces_normal;
	
	vec3 normal_map (vec2 uv) {
		vec3 norm_sampl = texture(surfaces_normal, vec3(uv, v.tex_id)).rgb;
		norm_sampl = pow(norm_sampl, vec3(1.0/2.2)); // is this right?
		
		norm_sampl.y = 1.0 - norm_sampl.y;
		norm_sampl = normalize(norm_sampl * 2.0 - 1.0);
		
		vec3 normal    = v.norm;
		vec3 tangent   = vec3(1.0, 0.0, 0.0);
		vec3 bitangent = vec3(0.0, 1.0, 0.0);
		
		mat3 mat = mat3(tangent, bitangent, normal);
		return mat * norm_sampl;
	}
	
	out vec4 frag_col;
	void main () {
		vec2 uv = v.pos.xy * 0.5;
		vec3 norm = normal_map(uv);
		vec4 col = texture(surfaces_color, vec3(uv, v.tex_id));
		//col = vec4(1,1,1,1);
		
		col.rgb *= simple_lighting(v.pos, norm);
		
		frag_col = col;
	}
#endif
