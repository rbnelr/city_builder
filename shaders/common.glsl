
#if defined(_VERTEX)
	#define VS2FS out Vertex v;
#elif defined(_FRAGMENT)
	#define VS2FS in Vertex v;
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

/*
struct Lighting {
	vec3 sun_dir;
	
	vec3 sun_col;
	vec3 sky_col;
	vec3 skybox_bottom_col;
	vec3 fog_col;
	
	float fog_base;
	float fog_falloff;
};*/

// layout(std140, binding = 0) only in #version 420
layout(std140, binding = 0) uniform Common {
	View3D view;
	//Lighting lighting;
};

//#include "dbg_indirect_draw.glsl"

/*
float fresnel (float dotVN, float F0) {
	float x = clamp(1.0 - dotVN, 0.0, 1.0);
	float x2 = x*x;
	return F0 + ((1.0 - F0) * x2 * x2 * x);
}
float fresnel_roughness (float dotVN, float F0, float roughness) {
	float x = clamp(1.0 - dotVN, 0.0, 1.0);
	float x2 = x*x;
	return F0 + ((max((1.0 - roughness), F0) - F0) * x2 * x2 * x);
}

vec3 simple_lighting (vec3 pos, vec3 normal) {
	float stren = sun_strength();
	
	vec3 sun = lighting.sun_col - atmos_scattering();
	
	//vec3 dir = normalize(pos - view.cam_pos);
	
	float d = max(dot(lighting.sun_dir, normal), 0.0);
	vec3 diffuse =
		(stren        ) * sun * d +
		(stren + 0.008) * lighting.sky_col*0.3;
	
	return vec3(diffuse);
}
*/