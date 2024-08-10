#version 430
#include "common.glsl"
#include "gbuf.glsl"
#include "entity_instances.glsl"

VS2FS Vertex {
	vec3 world_pos;
	vec3 world_normal;
	vec2 uv;
	
	flat vec3 tint_col;
	flat int tex_id;
} v;

#ifdef _VERTEX

layout(location = 0) in vec3  mesh_pos;
layout(location = 1) in vec3  mesh_normal;
layout(location = 2) in vec2  mesh_uv;
layout(location = 3) in uint  mesh_boneID;

//layout(location = 4) in int   mesh_id;
layout(location = 5) in int   instance_id;
layout(location = 6) in int   light_id;
layout(location = 7) in int   tex_id;
layout(location = 8) in vec3  instance_pos;
layout(location = 9) in vec3  instance_col;
// could get these like this as well, if ssbo is needed anyway for bone array like access
// -> instance[gl_InstanceID].posx, instance[gl_InstanceID].posy ...

void main () {
	mat4x3 bone_transform = mat4x3(instance[instance_id].bone_rot[mesh_boneID]);
	
	v.world_pos    = (bone_transform * vec4(mesh_pos, 1.0)).xyz + instance_pos;
	v.world_normal = mat3(bone_transform) * mesh_normal;
	
	v.uv           = mesh_uv;
	v.tint_col     = instance_col;
	v.tex_id       = tex_id;
	
	gl_Position = view.world2clip * vec4(v.world_pos, 1.0);
}

#endif

#ifdef _FRAGMENT
	uniform sampler2D tex;
	
	GBUF_OUT
	void main () {
		vec4 diff = texture(bindless_tex(v.tex_id), v.uv);
		vec3 TRM = texture(bindless_tex(v.tex_id+1), v.uv).rgb;
		
		// PBR.R as tint channel to tint albedo with instance color
		diff.rgb *= mix(vec3(1.0), v.tint_col, TRM.r);
		
		frag_col   = diff;
		frag_emiss = vec4(0,0,0,1);
		frag_norm  = vec4(v.world_normal, 1.0);
		frag_pbr   = vec4(TRM.gb, 0,1);
	}
#endif
