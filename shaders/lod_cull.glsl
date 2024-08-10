#version 430
layout(local_size_x = GROUPSZ) in;
#include "common.glsl"

#include "entity_instances.glsl"

// assume uint == uint32_t

struct MeshInfo {
	uint mesh_lod_id; // index of MeshLodInfos [lods]
	uint lods;
};
struct MeshLodInfo {
	uint vertex_base;
	uint vertex_count;
	uint index_base;
	uint index_count;
};

layout(std430, binding = 3) writeonly restrict buffer CommandsBuf {
	glDrawElementsIndirectCommand cmd[];
};
layout(std430, binding = 4) readonly restrict buffer MeshInfoBuf {
	MeshInfo mesh_info[];
};
layout(std430, binding = 5) readonly restrict buffer MeshLodInfoBuf {
	MeshLodInfo mesh_lod_info[];
};

uniform float lod_bias = -3.0f;
uniform float lod_fac  = 0.45f;

uint pick_lod (vec3 obj_pos, uint lod_count) {
	float obj_lod_start = 32.0;
	float dist = max(distance(view.cam_pos, obj_pos) - obj_lod_start, 1.0);

	uint lod = uint(round(lod_fac * log2(dist) + lod_bias));
	lod = clamp(lod, 0u, lod_count-1u);
	return lod;
}

uniform uint instance_count;

void main () {
	// might want to do this once we want to stop rendering objects once they get to far away
	// ie. mostly props, citizens etc.
	//uint idx = atomicAdd(_dbgdrawbuf.lines.cmd.count, 2u);
	
	uint i = gl_GlobalInvocationID.x;
	if (i >= instance_count) return;
	
	vec3 pos = vec3(instance[i].posx, instance[i].posy, instance[i].posz);
	uint mesh_id = instance[i].mesh_id;
	MeshInfo info = mesh_info[mesh_id];
	
	uint lod = pick_lod(pos, info.lods);
	
	MeshLodInfo lod_info = mesh_lod_info[info.mesh_lod_id + lod];
	
	cmd[i].count = lod_info.index_count;
	cmd[i].primCount = 1u;
	cmd[i].firstIndex = lod_info.index_base;
	cmd[i].baseVertex = int(lod_info.vertex_base);
	cmd[i].baseInstance = i;
}
