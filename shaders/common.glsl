#extension GL_ARB_gpu_shader5 : enable
#extension GL_ARB_gpu_shader_int64 : enable

// VS2FS does not work when using other shader stages (GEOMERY SHADER)
// manaully put things in the particular shaders?
#if defined(_VERTEX)
	#define VS2FS out
#elif defined(_FRAGMENT)
	#define VS2FS in
#else
	#define VS2FS struct // this at least avoid generating confusing error, but creates garbage global struct instance that's not used in other stages
#endif

float map (float x, float a, float b) {
	return (x - a) / (b - a);
}

const float INF			= 340282346638528859811704183484516925440.0 * 2.0;
//const float INF			= 1. / 0.;

const float PI			= 3.1415926535897932384626433832795;

const float DEG2RAD		= 0.01745329251994329576923690768489;
const float RAD2DEG		= 57.295779513082320876798154814105;

const float SQRT_2	    = 1.4142135623730950488016887242097;
const float SQRT_3	    = 1.7320508075688772935274463415059;

const float HALF_SQRT_2	= 0.70710678118654752440084436210485;
const float HALF_SQRT_3	= 0.86602540378443864676372317075294;

const float INV_SQRT_2	= 0.70710678118654752440084436210485;
const float INV_SQRT_3	= 0.5773502691896257645091487805019;

struct View3D {
	// forward VP matrices
	mat4        world2clip;
	mat4        world2cam;
	mat4        cam2clip;

	// inverse VP matrices
	mat4        clip2world;
	mat4        cam2world;
	mat4        clip2cam;

	// more details for simpler calculations
	vec2        frust_near_size; // width and height of near plane (for casting rays for example)
	// near & far planes
	float       clip_near;
	float       clip_far;
	// just the camera center in world space
	vec3        cam_pos;
	// viewport width over height, eg. 16/9
	float       aspect_ratio;
	
	// viewport size (pixels)
	vec2        viewport_size;
	vec2        inv_viewport_size;
};

struct Lighting {
	float time_of_day;
	
	mat4 sun2world;
	mat4 world2sun;
	mat4 moon2world;
	mat4 world2moon;
	mat4 solar2world;
	mat4 world2solar;
	
	vec3 sun_col;
	vec3 sky_col;
	vec3 skybox_bottom_col;
	vec3 fog_col;
	
	float fog_base;
	float fog_falloff;
	
	float clouds_z;      // cloud height in m
	float clouds_sz;     // cloud texture size in m
	vec2  clouds_offset; // cloud uv offset for movement (wraps in [0,1))
	vec2  clouds_vel;    // current cloud velocity in m/s
};

// layout(std140, binding = 0) only in #version 420
layout(std140, binding = 0) uniform Common {
	View3D view;
	Lighting lighting;
};

// Bindless textures
struct BindlessTextureEntry {
	uint64_t handles[4];
	float uv_scale;
	float _pad;
};
layout(std430, binding = 0) buffer BindlessTextures {
	BindlessTextureEntry lut[];
} bindless_LUT;

sampler2D bindless_tex (int tex_id, int slot) {
	return sampler2D(bindless_LUT.lut[tex_id].handles[slot]);
}
vec4 bindless_tex_scaled (int tex_id, int slot, vec2 uv) {
	BindlessTextureEntry tex = bindless_LUT.lut[tex_id];
	return texture(sampler2D(tex.handles[slot]), uv * tex.uv_scale);
}


struct CubemapFace {
	vec3 world_dir; // direction this cubemap face is in
	vec3 world_u;   // where the cubemap face U points in the world
	vec3 world_v;   // where the cubemap face V points in the world
};
const CubemapFace _cubemap_faces[6] = {
	// GL_TEXTURE_CUBE_MAP_POSITIVE_X, right in opengl and in my world
	CubemapFace( vec3(+1,0,0), vec3(0,-1,0), vec3(0,0,+1) ),
	// GL_TEXTURE_CUBE_MAP_NEGATIVE_X
	CubemapFace( vec3(-1,0,0), vec3(0,+1,0), vec3(0,0,+1) ),
	// GL_TEXTURE_CUBE_MAP_POSITIVE_Y, supposedly up in opengl, but nsight puts it on the bottom, so this face becomes down for me
	CubemapFace( vec3(0,0,-1), vec3(+1,0,0), vec3(0,+1,0) ),
	// GL_TEXTURE_CUBE_MAP_NEGATIVE_Y
	CubemapFace( vec3(0,0,+1), vec3(+1,0,0), vec3(0,-1,0) ),
	// GL_TEXTURE_CUBE_MAP_POSITIVE_Z // forwad in opengl
	CubemapFace( vec3(0,+1,0), vec3(+1,0,0), vec3(0,0,+1) ),
	// GL_TEXTURE_CUBE_MAP_NEGATIVE_Z
	CubemapFace( vec3(0,-1,0), vec3(-1,0,0), vec3(0,0,+1) ),
};
// Map opengl/nsight's cubemap orientation to my z-up system
vec4 readCubemap (samplerCube cubemap, vec3 dir_world) {
	return texture(cubemap, vec3(dir_world.x, -dir_world.z, dir_world.y));
}
vec4 readCubemapLod (samplerCube cubemap, vec3 dir_world, float lod) {
	return textureLod(cubemap, vec3(dir_world.x, -dir_world.z, dir_world.y), lod);
}

#include "dbg_indirect_draw.glsl"

float sin_hash (vec2 x) {
	return fract(sin(dot(x, vec2(12.9898, 78.233))) * 43758.5453);
}

mat3 mat_rotateX (float rot) {
	float s = sin(rot);
	float c = cos(rot);
	return mat3( // Column major for some insane reason!
		 1,  0,  0,
		 0,  c,  s,
		 0, -s,  c
	);
}
mat3 mat_rotateY (float rot) {
	float s = sin(rot);
	float c = cos(rot);
	return mat3( // Column major for some insane reason!
		 c,  0, -s,
		 0,  1,  0,
		 s,  0,  c
	);
}
mat3 mat_rotateZ (float rot) {
	float s = sin(rot);
	float c = cos(rot);
	return mat3( // Column major for some insane reason!
		 c,  s,  0,
		-s,  c,  0,
		 0,  0,  1
	);
}

mat3 mat_rotate_eulerXY (float x, float y) {
	float sx = sin(x);
	float cx = cos(x);
	
	float sy = sin(y);
	float cy = cos(y);
	
	return mat3(
		   cy,    0,    -sy, 
		sy*sx,  +cx,  cy*sx, 
		sy*cx,  -sx,  cy*cx 
	);
}

mat3 mat_rotate_eulerXYZ (float x, float y, float z) {
	float sx = sin(x);
	float cx = cos(x);
	
	float sy = sin(y);
	float cy = cos(y);
	
	float sz = sin(z);
	float cz = cos(z);
	
	return mat3(
		             cz*cy,               sz*cy,    -sy, 
		cz*(sy*sx) - sz*cx,  sz*(sy*sx) + cz*cx,  cy*sx, 
		cz*(sy*cx) + sz*sx,  sz*(sy*cx) - cz*sx,  cy*cx 
	);
}

// TODO: not sure if this code is wrong, or my curb normals are wrong?
//  hard to debug normals visually, maybe with indirect dbg arrows?
vec3 normal_map (vec3 normal, vec3 tangent, vec3 norm_sampl) {
	// generate bitangent vector orthogonal to both normal and tangent
	vec3 bitangent = cross(normal, tangent);
	// regenerate tangent vector in case it was not orthogonal to normal
	tangent = cross(bitangent, normal);
	// build matrix that does tangent space -> world space
	mat3 TBN = mat3(normalize(tangent), normalize(bitangent), normal);
	
	//norm_sampl *= 4.0;
	norm_sampl = pow(norm_sampl, vec3(1.0/2.2)); // gamma correct, is this right?
	
	norm_sampl.y = 1.0 - norm_sampl.y;
	norm_sampl = normalize(norm_sampl * 2.0 - 1.0);
	
	// bring normal map vector (tangent space) into world space
	return TBN * norm_sampl;
}

// standard dodgy way of coming up with a tangent space based on normal
//  if no tanget vector is available
mat3 dodgy_TBN (vec3 normal) {
	// up is either up if normal to side, or alternatively to the right (this generate discontinuity, which is why this is dodgy) 
	vec3 up = abs(normal.z) < 0.999 ? vec3(0,0,1) : vec3(1,0,0);
	// tangent points to right of surface
	vec3 tangent = normalize( cross(up, normal) );
	// bitangent points towards up
	vec3 bitangent = cross(normal, tangent);
	// regenerate tangent vector in case it was not orthogonal to normal
	tangent = cross(bitangent, normal);
	
	// build matrix that does tangent space -> world space
	return mat3(normalize(tangent), normalize(bitangent), normal);
}

uniform sampler2D grid_tex;
uniform sampler2D contours_tex;

vec3 overlay_grid (vec3 col, vec3 pix_pos) {
	vec2 uv = pix_pos.xy / 80.0;
	float c = texture(grid_tex, uv).r;
	return mix(col, vec3(c), vec3(0.5));
}

vec3 overlay_countour_lines (vec3 col, vec3 pix_pos) {
	float height = pix_pos.z / 0.1;
	float c = texture(contours_tex, vec2(height, 0.5)).r;
	return mix(col, vec3(c), vec3(0.5));
}
