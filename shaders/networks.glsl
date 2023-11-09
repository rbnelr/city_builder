#version 430
#include "common.glsl"
#include "gbuf.glsl"

struct Vertex {
	vec3 pos;
	vec3 norm;
	vec3 tang;
	vec2 uv;
	float tex_id;
};
VS2FS

#ifdef _VERTEX
	layout(location = 0) in vec3  pos;
	layout(location = 1) in vec3  norm;
	layout(location = 2) in vec3  tang;
	layout(location = 3) in vec2  uv;
	layout(location = 4) in float tex_id;
	
	void main () {
		gl_Position = view.world2clip * vec4(pos, 1.0);
		
		v.pos  = pos;
		v.tex_id = tex_id;
		
		vec2 world_uv = pos.xy * 0.5;
		if (tex_id < 0) {
			v.uv = uv;
			v.norm = norm;
			v.tang = tang;
		}
		else {
			v.uv = world_uv;
			v.norm = vec3(0,0,1);
			v.tang = vec3(1,0,0);
		}
	}
#endif
#ifdef _FRAGMENT
	uniform sampler2D test_color;
	uniform sampler2D test_normal;
	
	uniform sampler2DArray surfaces_color;
	uniform sampler2DArray surfaces_normal;
	
	vec3 normal_map (vec2 uv) {
		vec3 normal    = v.norm;
		vec3 tangent   = v.tang;
		vec3 bitangent = cross(normal, tangent); // generate bitangent vector orthogonal to both normal and tangent
		tangent = cross(bitangent, normal); // regenerate tangent vector in case it was not orthogonal to normal
	
		mat3 TBN = mat3(normalize(tangent), normalize(bitangent), normal);
		
		
		vec3 norm_sampl = v.tex_id < 0 ?
			texture(test_normal, uv).rgb :
			texture(surfaces_normal, vec3(uv, v.tex_id)).rgb;
		//norm_sampl *= 4.0;
		norm_sampl = pow(norm_sampl, vec3(1.0/2.2)); // is this right?
		
		norm_sampl.y = 1.0 - norm_sampl.y;
		norm_sampl = normalize(norm_sampl * 2.0 - 1.0);
		
		return TBN * norm_sampl;
	}
	
	GBUF_OUT
	void main () {
		
		vec3 norm = normal_map(v.uv);
		//vec3 norm = v.norm;
		vec3 col = v.tex_id < 0 ?
			texture(test_color, v.uv).rgb :
			texture(surfaces_color, vec3(v.uv, v.tex_id)).rgb;
		//col = vec4(1,1,1,1);
		
		frag_col = vec4(col, 1.0);
		frag_norm = vec4(norm, 1.0);
	}
#endif
