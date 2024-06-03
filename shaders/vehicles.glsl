#version 430
#include "common.glsl"
#include "gbuf.glsl"

VS2FS Vertex {
	vec3 world_pos;
	vec3 world_normal;
	vec2 uv;
	
	flat vec3 col;
} v;

#ifdef _VERTEX

layout(location = 0) in vec3  mesh_pos;
layout(location = 1) in vec3  mesh_normal;
layout(location = 2) in vec2  mesh_uv;
layout(location = 3) in uint  mesh_boneID;
//layout(location = 4) in int   mesh_id;
layout(location = 5) in vec3  instance_pos;
layout(location = 6) in vec3  instance_rot;
layout(location = 7) in vec3  instance_col;
layout(location = 7) in float instance_steer_ang;

uniform float time = 0;
uniform mat4 _mats[5];
uniform mat4 _mats_inv[5];

void main () {
	// heading rotation
	mat4 rot_mat = mat4(mat_rotateZ(instance_rot.z));
	
	{
		mat4 bone_rot;
		if (mesh_boneID == 0) {
			// heading rotation
			bone_rot = mat4(mat_rotateX(instance_rot.x) * mat_rotateZ(-instance_rot.y));
		}
		else {
			// 
			bone_rot = mat4(mat_rotateZ(-time * 360.0*DEG2RAD));
		}
		
		mat4 mesh2bone = _mats[mesh_boneID];
		mat4 bone2mesh = _mats_inv[mesh_boneID];
		//rot_mat = rot_mat * (bone2mesh * bone_rot * mesh2bone);
		rot_mat = rot_mat * (bone2mesh * bone_rot * mesh2bone);
	}
	
	v.world_pos    = (rot_mat * vec4(mesh_pos, 1.0)).xyz + instance_pos;
	v.world_normal = mat3(rot_mat) * mesh_normal;
	
	v.uv           = mesh_uv;
	v.col          = instance_col;
	
	gl_Position = view.world2clip * vec4(v.world_pos, 1.0);
}

#endif

#ifdef _FRAGMENT
	uniform sampler2D tex;
	
	uniform float time = 0;
	
	GBUF_OUT
	void main () {
		frag_col = texture(tex, v.uv) * vec4(v.col, 1.0);
		frag_norm = vec4(v.world_normal, 1.0);
	}
#endif
