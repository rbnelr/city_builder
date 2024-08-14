#version 430
#include "common.glsl"

VS2FS Vertex {
	flat vec3 inst_pos;
	flat mat3 world2local;
	
	//vec3 test_pos;
	//vec3 normal;
	//vec2 test_uv;
} v;

#ifdef _VERTEX
	layout(location = 0) in vec3  mesh_pos;
	//layout(location = 1) in vec3  mesh_normal;
	layout(location = 2) in vec3  instance_pos;
	layout(location = 3) in float instance_rot;
	layout(location = 4) in vec3  instance_size;
	
	void main () {
		mat3 rot_mat = mat_rotateZ(instance_rot);
	
		vec3 pos_world = (rot_mat * (instance_size * mesh_pos)) + instance_pos;
		//vec3 norm_world = rot_mat * -mesh_normal; // flip normal here for simplicity, but could do this in blender as well
		
		gl_Position = view.world2clip * vec4(pos_world, 1.0);
		
		vec3 inv_size = 1.0 / instance_size;
		
		v.inst_pos = instance_pos;
		
		mat3 mat;
		mat[0] = rot_mat * vec3(inv_size.x, 0,0);
		mat[1] = rot_mat * vec3(0, inv_size.y,0);
		mat[2] = rot_mat * vec3(0,0, inv_size.z);
		v.world2local = transpose(mat);
		
		//v.test_pos = pos_world;
		//v.normal = norm_world;
		//v.test_uv = mesh_pos.xy + 0.5;
	}
#endif
#ifdef _FRAGMENT
	#define GBUF_IN 1
	#include "gbuf.glsl"
	
	//uniform vec4 pbr = vec4(0.95, 0.0, 0,1);
	//uniform sampler2D test_tex;
	
	GBUF_OUT
	void main () {
		vec3 gbuf_pos_world;
		if (!decode_gbuf_pos(gbuf_pos_world))
			discard;
			
		// substract position then inverse rotate and scale, this is cheaper than full 4x4 matrix
		vec3 xyz = v.world2local * (gbuf_pos_world - v.inst_pos);
		if (abs(xyz.x) > 0.5 || abs(xyz.y) > 0.5 || abs(xyz.z) > 0.5)
			discard; // discard pixels outside decal volume
		
		// reset depth to infinitely far away
		gl_FragDepth = 0.0; // is this always correct?
		
		/*
		vec3 N = abs(v.normal);
		vec2 uv = N.z > N.x && N.z > N.y ? v.test_pos.xy : (N.x > N.y ? v.test_pos.yz : v.test_pos.xz);
		vec4 col = texture(test_tex, uv / 32.0);
		
		frag_col  = col;
		frag_emiss = vec4(vec3(0), 1);
		frag_norm = vec4(v.normal, 1);
		frag_pbr = vec4(pbr.rgb, 1);
		
		//gl_FragDepth = 0.0;
		return;*/
	}
#endif
