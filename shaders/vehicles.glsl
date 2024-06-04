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

mat4x3 get_mat () {
	VehicleInstance i = instance[instance_id];
	
	float a0 = i.bone_rot[mesh_boneID][0][0];
	float a1 = i.bone_rot[mesh_boneID][0][1];
	float a2 = i.bone_rot[mesh_boneID][0][2];
	float b0 = i.bone_rot[mesh_boneID][1][0];
	float b1 = i.bone_rot[mesh_boneID][1][1];
	float b2 = i.bone_rot[mesh_boneID][1][2];
	float c0 = i.bone_rot[mesh_boneID][2][0];
	float c1 = i.bone_rot[mesh_boneID][2][1];
	float c2 = i.bone_rot[mesh_boneID][2][2];
	float d0 = i.bone_rot[mesh_boneID][3][0];
	float d1 = i.bone_rot[mesh_boneID][3][1];
	float d2 = i.bone_rot[mesh_boneID][3][2];
	
	return mat4x3(
		a0,a1,a2,
		b0,b1,b2,
		c0,c1,c2,
		d0,d1,d2
	);
}

void main () {
	//mat4 bone_transform = mat4(instance[instance_id].bone_rot[mesh_boneID]);
	mat4x3 bone_transform = get_mat();
	
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
