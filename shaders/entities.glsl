#version 430
#include "common.glsl"
#include "entity_instances.glsl"
#include "procedural_sky.glsl"

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

#if TRAFFIC_SIGNALS
// TODO: maybe this would be better off as an SSBO after all?
// have EntityRenderer allow specifiying ssbos? make template have Asset (for mesh), Instance, and then just generic number of ssbos based on sizes?
layout(location =  9) in vec3 inst_traffic_col1;
layout(location = 10) in vec3 inst_traffic_col2;
layout(location = 11) in vec3 inst_traffic_col3;
#endif

#if LAMPS
	// TODO: Turn point lights off as well!, but need common id that gets passed into both, to correctly identify
	bool is_lamp_on () {
		float offs = sin_hash(inst_pos.xy) * 0.05;
		
		return sun_strength() + offs < 0.20;
	}

	vec3 get_emissive () {
		return is_lamp_on() ? vec3(0.99,0.98,0.94)*0.3 : vec3(0,0,0); // TODO: allow different lamp colors
	}
#elif TRAFFIC_SIGNALS
	vec3 get_emissive () {
		if      (mesh_vtxGrpID == 1) return inst_traffic_col1 * 2.0;
		else if (mesh_vtxGrpID == 2) return inst_traffic_col2 * 2.0;
		else                         return inst_traffic_col3 * 2.0;
	}
#else
	vec3 get_emissive () {
		return vec3(0,0,0);
	}
#endif

void main () {
	mat3 rot_mat = mat_rotateZ(inst_rot);
	
	v.world_pos    = rot_mat * mesh_pos + inst_pos;
	v.world_normal = rot_mat * mesh_normal;
	
	v.uv           = mesh_uv;
	v.emissive_col = get_emissive();
	v.tex_id       = inst_tex_id;
	
	gl_Position = view.world2clip * vec4(v.world_pos, 1.0);
}

#endif

#ifdef _FRAGMENT
	#include "gbuf.glsl"
	
	GBUF_OUT
	void main () {
		GBUF_HANDLE_WIREFRAME
		
		vec4 col = texture(bindless_tex(v.tex_id, 0), v.uv);
		vec3 ERM = texture(bindless_tex(v.tex_id, 2), v.uv).rgb;
		
		frag_col   = col;
		// PBR.R as emissive channel
		frag_emiss = vec4(v.emissive_col * ERM.r * lighting.exposure, 1);
		frag_norm  = vec4(v.world_normal, 1);
		frag_pbr   = vec4(ERM.gb, 0,1);
	}
#endif
