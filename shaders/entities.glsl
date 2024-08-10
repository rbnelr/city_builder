#version 430
#include "common.glsl"
#include "gbuf.glsl"
#include "entity_instances.glsl"

VS2FS Vertex {
	vec3 world_pos;
	vec3 world_normal;
	vec2 uv;
	
	flat vec3 emissive_col;
	flat int tex_id;
} v;

#ifdef _VERTEX

layout(location = 0) in vec3  mesh_pos;
layout(location = 1) in vec3  mesh_normal;
layout(location = 2) in vec2  mesh_uv;
layout(location = 3) in int   mesh_vtxGrpID;
//layout(location = 4) in int   inst_mesh_id;
layout(location = 5) in int   inst_tex_id;
layout(location = 6) in vec3  inst_pos;
layout(location = 7) in float inst_rot;
layout(location = 8) in vec3  inst_tint; // TODO: not being used current?

void main () {
	mat3 rot_mat = mat_rotateZ(inst_rot);
	
	v.world_pos    = rot_mat * mesh_pos + inst_pos;
	v.world_normal = rot_mat * mesh_normal;
	
	v.uv           = mesh_uv;
	if (mesh_vtxGrpID == 0)     v.emissive_col = vec3(1,0,0);
	else if (mesh_vtxGrpID== 1) v.emissive_col = vec3(0,1,0);
	else                        v.emissive_col = vec3(0,0,1);
	//v.emissive_col = lights[light_id + mesh_vtxGrpID].rgb;
	v.tex_id       = inst_tex_id;
	
	gl_Position = view.world2clip * vec4(v.world_pos, 1.0);
}

#endif

#ifdef _FRAGMENT
	GBUF_OUT
	void main () {
		vec4 col = texture(bindless_tex(v.tex_id), v.uv);
		vec3 ERM = texture(bindless_tex(v.tex_id+1), v.uv).rgb;
		
		frag_col   = col;
		// PBR.R as emissive channel
		frag_emiss = vec4(v.emissive_col * ERM.r, 1);
		frag_norm  = vec4(v.world_normal, 1);
		frag_pbr   = vec4(ERM.gb, 0,1);
	}
#endif
