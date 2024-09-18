#version 430
#include "common.glsl"

VS2FS Vertex {
	flat vec3 pos;
	flat mat3 world2decal;
	
	flat int  tex_id;
	flat vec2 uv_scale;
	flat vec4 col;
} v;

#ifdef _VERTEX
	layout(location = 0) in vec3  mesh_pos;
	layout(location = 1) in vec3  instance_pos;
	layout(location = 2) in float instance_rot;
	layout(location = 3) in vec3  instance_size;
	layout(location = 4) in int   instance_tex_id;
	layout(location = 5) in vec2  instance_uv_scale;
	layout(location = 6) in vec4  instance_col;
	
	void main () {
		mat3 rot_mat = mat_rotateZ(instance_rot);
	
		vec3 pos_world = (rot_mat * (instance_size * mesh_pos)) + instance_pos;
		
		gl_Position = view.world2clip * vec4(pos_world, 1.0);
		
		vec3 inv_size = 1.0 / instance_size;
		
		v.pos = instance_pos;
		mat3 mat;
		// why are we doing a normal rotation and a inverse scale, and then transpose(mat)?
		mat[0] = rot_mat * vec3(inv_size.x, 0,0);
		mat[1] = rot_mat * vec3(0, -inv_size.y,0); // flip y coords to get uv coord flipped, since all textures are flipped!
		mat[2] = rot_mat * vec3(0,0, inv_size.z);
		v.world2decal = transpose(mat); // cheap inverse, valid for rotation/scale matrix
		
		v.tex_id = instance_tex_id;
		v.uv_scale = instance_uv_scale;
		v.col = instance_col;
	}
#endif
#ifdef _FRAGMENT
	#define GBUF_IN 1
	#include "gbuf.glsl"
	
	uniform sampler2D cracks;
	uniform vec4 pbr = vec4(0.3,0.0, 0,1);
	
	GBUF_OUT
	void main () {
		GBUF_HANDLE_WIREFRAME
		
		vec3 gbuf_pos_world;
		if (!decode_gbuf_pos(gbuf_pos_world))
			discard;
			
		// substract position then inverse rotate and scale, this is cheaper than full 4x4 matrix
		vec3 xyz = v.world2decal * (gbuf_pos_world - v.pos);
		if (abs(xyz.x) > 0.5 || abs(xyz.y) > 0.5 || abs(xyz.z) > 0.5)
			discard; // discard pixels outside decal volume
		
		vec3 norm = vec3(v.world2decal[0].z, v.world2decal[1].z, v.world2decal[2].z);
		vec2 uv = xyz.xy + vec2(0.5);
		uv *= v.uv_scale;
		
		// TODO: could use xyz.z to blend decal in out out at the vertical edges
		vec4 col = v.col;
		col.a *= texture(bindless_tex(v.tex_id, 0), uv).r;
		
		float alpha = col.a;
		alpha *= texture(cracks, gbuf_pos_world.xy * 0.2).r;
		// funky math because I couldn't be bothered to change the texture (TODO: ??)
		alpha = clamp(map(alpha - 0.12, 0.0, 0.10), 0.0, 1.0);
		col.a = alpha * 0.8;
		
		col.rgb *= 0.7;
		
		frag_col  = col;
		frag_emiss = vec4(vec3(0), col.a); // cover up emissive, decals currently never emissive
		frag_norm = vec4(norm, 0.7 * col.a);
		frag_pbr = vec4(pbr.rgb, col.a);
		
		//frag_col = vec4(1,0,0,1);
	}
#endif
