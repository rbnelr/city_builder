#version 430
#include "common.glsl"
#include "gbuf.glsl"
#include "entity_instances.glsl"

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
layout(location = 5) in int   instance_id;
layout(location = 6) in vec3  instance_pos;
layout(location = 7) in vec3  instance_col;
// could get these like this as well, if ssbo is needed anyway for bone array like access
// -> instance[gl_InstanceID].posx, instance[gl_InstanceID].posy ...

uniform float time = 0;
uniform mat4 _mats[5];
uniform mat4 _mats_inv[5];

void main () {
	// heading rotation
	//mat4 rot_mat = mat4(mat_rotateZ(instance_rot.z));
	//
	//instance[gl_InstanceID].posx
	//
	//{
	//	mat4 bone_rot;
	//	if (mesh_boneID == 0) {
	//		// heading rotation
	//		bone_rot = mat4(mat_rotateX(instance_rot.x) * mat_rotateZ(-instance_rot.y));
	//	}
	//	else {
	//		// 
	//		bone_rot = mat4(mat_rotateZ(-time * 360.0*DEG2RAD));
	//	}
	//	
	//	mat4 mesh2bone = _mats[mesh_boneID];
	//	mat4 bone2mesh = _mats_inv[mesh_boneID];
	//	//rot_mat = rot_mat * (bone2mesh * bone_rot * mesh2bone);
	//	rot_mat = rot_mat * (bone2mesh * bone_rot * mesh2bone);
	//}
	
	float rotX = instance[instance_id].bone_rot[mesh_boneID*3  ];
	float rotY = instance[instance_id].bone_rot[mesh_boneID*3+1];
	float rotZ = instance[instance_id].bone_rot[mesh_boneID*3+2];
	
	mat4 heading_rot = mat4(mat_rotateZ(rotZ));
	mat4 bone_rot = mat4(mat_rotate_eulerXY(rotX, rotY));
	
	mat4 mesh2bone = _mats[mesh_boneID];
	mat4 bone2mesh = _mats_inv[mesh_boneID];
	
	mat4 bone_transform = heading_rot * (bone2mesh * (bone_rot * mesh2bone));
	
	v.world_pos    = (bone_transform * vec4(mesh_pos, 1.0)).xyz + instance_pos;
	v.world_normal = mat3(bone_transform) * mesh_normal;
	
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
