//// Debug Line Rendering

struct glDrawArraysIndirectCommand {
	uint count;
	uint instanceCount;
	uint first;
	uint baseInstance;
};

struct glDrawElementsIndirectCommand {
	uint  count;
	// not a primitive count?? glDrawElementsInstancedBaseVertexBaseInstance
	// >glDrawElementsInstancedBaseVertexBaseInstance: primcount: Specifies the number of instances of the indexed geometry that should be drawn.
	uint  primCount;
	uint  firstIndex;
	int   baseVertex;
	uint  baseInstance;
};

struct IndirectLineVertex {
	vec4 pos;
	vec4 col;
};
//struct IndirectWireInstace {
//	vec4 pos;
//	vec4 size;
//	vec4 col;
//};

// Large SSBO sizes cause extremely long shader compile times?
#define _INDIRECT_BUFSZ (4096*2)
struct IndirectLines {
	glDrawArraysIndirectCommand cmd;
	IndirectLineVertex vertices[_INDIRECT_BUFSZ];
};
//struct IndirectWireInstances {
//	glDrawElementsIndirectCommand cmd;
//	uint _pad[3]; // padding for std430 alignment
//	IndirectWireInstace vertices[_INDIRECT_BUFSZ];
//};

layout(std430, binding = 2) restrict buffer IndirectBuffer {
	bool update;
	
	IndirectLines         lines;
	//IndirectWireInstances wire_cubes;
	//IndirectWireInstances wire_spheres;
} _dbgdrawbuf;

//void dbgdraw_clear () {
//	_dbgdrawbuf.lines       .cmd.count = 0;
//	_dbgdrawbuf.wire_cubes  .cmd.primCount = 0;
//	_dbgdrawbuf.wire_spheres.cmd.primCount = 0;
//}
void dbgdraw_vector (vec3 pos, vec3 dir, vec4 col) {
	if (!_dbgdrawbuf.update) return;
	uint idx = atomicAdd(_dbgdrawbuf.lines.cmd.count, 2u);
	if (idx >= _INDIRECT_BUFSZ) return;
	
	_dbgdrawbuf.lines.vertices[idx++] = IndirectLineVertex( vec4(pos      , 0), col );
	_dbgdrawbuf.lines.vertices[idx++] = IndirectLineVertex( vec4(pos + dir, 0), col );
}
void dbgdraw_point (vec3 pos, float r, vec4 col) {
	if (!_dbgdrawbuf.update) return;
	uint idx = atomicAdd(_dbgdrawbuf.lines.cmd.count, 6u);
	if (idx >= _INDIRECT_BUFSZ) return;
	
	_dbgdrawbuf.lines.vertices[idx++] = IndirectLineVertex( vec4(pos - vec3(r,0,0), 0), col );
	_dbgdrawbuf.lines.vertices[idx++] = IndirectLineVertex( vec4(pos + vec3(r,0,0), 0), col );
	
	_dbgdrawbuf.lines.vertices[idx++] = IndirectLineVertex( vec4(pos - vec3(0,r,0), 0), col );
	_dbgdrawbuf.lines.vertices[idx++] = IndirectLineVertex( vec4(pos + vec3(0,r,0), 0), col );
	
	_dbgdrawbuf.lines.vertices[idx++] = IndirectLineVertex( vec4(pos - vec3(0,0,r), 0), col );
	_dbgdrawbuf.lines.vertices[idx++] = IndirectLineVertex( vec4(pos + vec3(0,0,r), 0), col );
}

//void dbgdraw_wire_cube (vec3 pos, vec3 size, vec4 col) { // size is edge length aka diameter
//	uint idx = atomicAdd(_dbgdrawbuf.wire_cubes.cmd.primCount, 1u);
//	if (idx >= _INDIRECT_BUFSZ) return;
//	_dbgdrawbuf.wire_cubes.vertices[idx] = IndirectWireInstace( vec4(pos, 0), vec4(size, 0), col);
//}
//void dbgdraw_wire_sphere (vec3 pos, vec3 size, vec4 col) { // size is sphere diameter (not radius)
//	uint idx = atomicAdd(_dbgdrawbuf.wire_spheres.cmd.primCount, 1u);
//	if (idx >= _INDIRECT_BUFSZ) return;
//	_dbgdrawbuf.wire_spheres.vertices[idx] = IndirectWireInstace( vec4(pos, 0), vec4(size, 0), col);
//}


void dbgdraw_visualize_normals (vec3 pos_world, vec3 norm_world) {
	if (distance(pos_world, view.cam_pos) < 5.0) {
		dbgdraw_vector(pos_world, norm_world * 0.1, vec4(0,0,1,1));
	}
}
void dbgdraw_visualize_normal_tangent (vec3 pos_world, vec3 norm_world, vec3 tangent) {
	if (distance(pos_world, view.cam_pos) < 5.0) {
		dbgdraw_vector(pos_world, norm_world * 0.1, vec4(0,0,1,1));
		dbgdraw_vector(pos_world, tangent * 0.1, vec4(1,0,0,1));
	}
}
#ifdef _FRAGMENT
void dbgdraw_visualize_normal_frag (vec3 pos_world, vec3 norm_world) {
	ivec2 xy = ivec2(gl_FragCoord.xy) % ivec2(1);
	if (xy.x == 0 && xy.y == 0 && distance(pos_world, view.cam_pos) < 10.0) {
		dbgdraw_vector(pos_world, norm_world * 0.1f, vec4(0,0,1,1));
	}
}
#endif
