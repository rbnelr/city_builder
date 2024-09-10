#version 430
#include "common.glsl"
#include "entity_instances.glsl"

VS2FS Vertex {
	vec3 model_pos;
	vec3 world_pos;
	vec3 world_normal;
	vec3 cam_normal;
	vec2 uv;
	
	flat vec3 tint_col;
	flat int tex_id;
	
	flat vec4 glow;
} v;

#ifdef _VERTEX

layout(location = 0) in vec3  mesh_pos;
layout(location = 1) in vec3  mesh_normal;
layout(location = 2) in vec2  mesh_uv;
layout(location = 3) in uint  mesh_boneID;
//layout(location = 4) in int   inst_mesh_id;
layout(location = 5) in int   inst_instance_id;
layout(location = 6) in int   inst_tex_id;
layout(location = 7) in vec3  inst_pos;
layout(location = 8) in vec3  inst_tint;
layout(location = 9) in vec4  inst_glow;
// could get these like this as well, if ssbo is needed anyway for bone array like access
// -> instance[gl_InstanceID].posx, instance[gl_InstanceID].posy ...

void main () {
	mat4x3 bone_transform = mat4x3(instance[inst_instance_id].bone_rot[mesh_boneID]);
	
	v.model_pos    = mesh_pos;
	v.world_pos    = (bone_transform * vec4(mesh_pos, 1.0)).xyz + inst_pos;
	v.world_normal = mat3(bone_transform) * mesh_normal;
	v.cam_normal   = mat3(view.world2cam) * mat3(bone_transform) * mesh_normal;
	
	v.uv           = mesh_uv;
	v.tint_col     = inst_tint;
	v.tex_id       = inst_tex_id;
	v.glow         = inst_glow;
	
	gl_Position = view.world2clip * vec4(v.world_pos, 1.0);
}

#endif

#ifdef _FRAGMENT
	#include "gbuf.glsl"
	
	uniform sampler2D tex;
	
	vec3 vehicle_emiss (vec3 glow_tex) {
		vec3 emiss = vec3(0.0);
		
		const vec3 front_lights_col = vec3(0.8, 0.8, 0.4) * 1.0;
		const vec3 rear_lights_col  = vec3(0.8, 0.02, 0.01) * 0.5;
		const vec3 brake_col        = vec3(0.8, 0.02, 0.01) * 0.5;
		const vec3 blinker_col      = vec3(0.8, 0.2, 0.01) * 1.4;
		
		vec3 lights_col = v.model_pos.x > 0.0 ? front_lights_col : rear_lights_col;
		emiss += v.glow.r * glow_tex.r * lights_col;
		
		emiss += v.glow.g * glow_tex.g * brake_col;
		
		float blinker_on = v.model_pos.y > 0.0 ? v.glow.z : v.glow.w;
		emiss += blinker_on * glow_tex.b * blinker_col;
		
		return emiss;// * emiss_normal_scale(v.cam_normal.z);
	}
	
	GBUF_OUT
	void main () {
		GBUF_HANDLE_WIREFRAME
		
		vec4 diff = texture(bindless_tex(v.tex_id, 0), v.uv);
		vec3 TRM  = texture(bindless_tex(v.tex_id, 2), v.uv).rgb;
		vec3 glow = texture(bindless_tex(v.tex_id, 3), v.uv).rgb;
		
		// PBR.R as tint channel to tint albedo with instance color
		diff.rgb *= mix(vec3(1.0), v.tint_col, TRM.r);
		
		vec3 emiss = vehicle_emiss(glow);
		
		frag_col   = diff;
		frag_emiss = vec4(emiss * lighting.exposure,1);
		frag_norm  = vec4(v.world_normal, 1.0);
		frag_pbr   = vec4(TRM.gb, 0,1);
	}
#endif
