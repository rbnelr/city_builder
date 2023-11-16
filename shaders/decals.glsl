#version 430
#include "common.glsl"

struct Vertex {
	vec3 pos;
	mat3 world2decal;
	
	vec4 col;
	float tex_id;
};
VS2FS

#ifdef _VERTEX
	layout(location = 0) in vec3  mesh_pos;
	layout(location = 1) in vec3  instance_pos;
	layout(location = 2) in vec3  instance_size;
	layout(location = 3) in float instance_rot;
	layout(location = 4) in vec4  instance_col;
	layout(location = 5) in float instance_tex_id;
	
	void main () {
		mat3 rot_mat = instance_rot_mat(instance_rot);
	
		vec3 pos_world = (rot_mat * (instance_size * mesh_pos)) + instance_pos;
		
		gl_Position = view.world2clip * vec4(pos_world, 1.0);
		
		vec3 inv_size = 1.0 / instance_size;
		
		v.pos = instance_pos;
		mat3 mat;
		mat[0] = rot_mat * vec3(inv_size.x, 0,0);
		mat[1] = rot_mat * vec3(0, inv_size.y,0);
		mat[2] = rot_mat * vec3(0,0, inv_size.z);
		v.world2decal = transpose(mat);
		
		v.col = instance_col;
		v.tex_id = instance_tex_id;
	}
#endif
#ifdef _FRAGMENT
	#define GBUF_IN 1
	#include "gbuf.glsl"
	
	uniform sampler2DArray turn_arrows;
	uniform sampler2D cracks;
	
	GBUF_OUT
	void main () {
		vec3 gbuf_pos_world;
		if (!decode_gbuf_pos(gbuf_pos_world))
			discard;
		
		vec3 xyz = v.world2decal * (gbuf_pos_world - v.pos);
		if (abs(xyz.x) > 0.5 || abs(xyz.y) > 0.5 || abs(xyz.z) > 0.5)
			discard;
		
		vec3 norm = vec3(v.world2decal[0].z, v.world2decal[1].z, v.world2decal[2].z);
		vec2 uv = xyz.xy + vec2(0.5);
		
		vec4 col = texture(turn_arrows, vec3(uv, v.tex_id)) * v.col;
		////col.a = 1.0;
		
		float alpha = col.a;
		alpha *= texture(cracks, gbuf_pos_world.xy * 0.2).r;
		
		alpha = clamp(map(alpha - 0.12, 0.0, 0.10), 0.0, 1.0);
		
		col.a = alpha * 0.8;
		col.rgb *= 0.7;
		
		frag_col  = col;
		frag_norm = vec4(norm, 0.7 * col.a);
		
		//frag_col = vec4(1,0,0,1);
	}
#endif
