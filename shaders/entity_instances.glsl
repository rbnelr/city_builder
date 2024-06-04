
// float x,y,z instead of vec3 to avoid alignment being applied (VBO instance buffer does not align, C++ float3 does not align and we can simply read as floats and turn them into a vec3
struct StaticEntityInstance {
	uint  mesh_id;
	float posx, posy, posz;
	float rotx;
	float colr, colg, colb;
};
struct VehicleInstance {
	uint  mesh_id;
	uint  instance_id;
	float posx, posy, posz;
	float colr, colg, colb;
	
	float bone_rot[5][4][3]; // mat4x3 is a float3x4 matrix because ogl uses wrong terminology
};
layout(std430, binding = 2) restrict buffer InstancesBuf {
	// TODO: we only rely on certain instance infos like position to compute lod
	// instead of needing to macro-ize shader for each instance vertex layout, could possibly read using some kind of buffer read where we just have a uniform int stride?
	INSTANCE_T instance[];
};

uniform uint instances_count;
