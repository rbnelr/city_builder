#version 430
layout(local_size_x = GROUPSZ) in;
#include "common.glsl"

struct glDrawElementsIndirectCommand {
	uint  count;
	uint  instanceCount;
	uint  firstIndex;
	int   baseVertex;
	uint  baseInstance;
};

layout(std430, binding = 2) restrict buffer IndirectBuffer {
	glDrawElementsIndirectCommand cmd[];
};

void main () {
	//uint idx = atomicAdd(_dbgdrawbuf.lines.cmd.count, 2u);
	
	uint i = gl_GlobalInvocationID.x;
	
	cmd[i].count = 500u * 3;
	cmd[i].instanceCount = 1u;
	cmd[i].firstIndex = 0u;
	cmd[i].baseVertex = 0;
	cmd[i].baseInstance = i;
}
