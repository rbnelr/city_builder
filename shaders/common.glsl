#extension GL_ARB_gpu_shader5 : enable
#extension GL_ARB_gpu_shader_int64 : enable

#if defined(_VERTEX)
	#define VS2FS out
#elif defined(_FRAGMENT)
	#define VS2FS in
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
	vec3 sun_dir;
	
	vec3 sun_col;
	vec3 sky_col;
	vec3 skybox_bottom_col;
	vec3 fog_col;
	
	float fog_base;
	float fog_falloff;
};

// layout(std140, binding = 0) only in #version 420
layout(std140, binding = 0) uniform Common {
	View3D view;
	Lighting lighting;
};

// Bindless textures
layout(std430, binding = 1) buffer BindlessTextures {
	uint64_t handles[];
} bindless_LUT;

sampler2D bindless_tex (int tex_id) {
	return sampler2D(bindless_LUT.handles[tex_id]);
}

//#include "dbg_indirect_draw.glsl"

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

uniform sampler2D clouds;

float sun_strength () {
	float a = map(-lighting.sun_dir.z, 0.5, -0.1);
	return smoothstep(1.0, 0.0, a);
}
vec3 atmos_scattering () {
	float a = map(-lighting.sun_dir.z, 0.5, -0.05);
	return vec3(0.0, 0.75, 0.8) * smoothstep(0.0, 1.0, a);
}
vec3 horizon (vec3 dir_world) {
	return vec3(clamp(map(dir_world.z, -0.10, 1.0), 0.05, 1.0));
}

vec3 get_skybox_light (vec3 view_point, vec3 dir_world) {
	float stren = sun_strength();
	
	vec3 col = lighting.sky_col * (stren + 0.008);
	
	vec3 sun = lighting.sun_col - atmos_scattering();
	sun *= stren;
	
	// sun
	float d = dot(dir_world, -lighting.sun_dir);
	
	const float sz = 500.0;
	float c = clamp(d * sz - (sz-1.0), 0.0, 1.0);
	
	col += sun * 20.0 * c;
	col *= horizon(dir_world);
	
	{
		const float clouds_z  = 3000.0;
		const float clouds_sz = 1024.0 * 32.0;
		
		float t = (clouds_z - view_point.z) / dir_world.z;
		if (dir_world.z > 0.0 && t >= 0.0) {
			vec3 pos = view_point + t * dir_world;
			
			vec4 c = texture(clouds, pos.xy / clouds_sz);
			
			col = mix(col.rgb, c.rgb * stren, vec3(c.a));
		}
	}
	
	float bloom_amount = max(dot(dir_world, -lighting.sun_dir) - 0.5, 0.0);
	col += bloom_amount * sun * 0.3;
	
	return col;
} 

vec3 apply_fog (vec3 pix_col, vec3 pix_pos) {
	// from https://iquilezles.org/articles/fog/
	// + my own research and derivation
	
	// TODO: properly compute extinction (out-scattering + absoption) and in-scattering
	// seperating in-scattering may allow for more blue tint at distance
	
	vec3 ray_cam = pix_pos - view.cam_pos;
	float dist = length(ray_cam);
	ray_cam = normalize(ray_cam);
	
	float stren = sun_strength();
	
	// exponential height fog parameters
	float a = lighting.fog_base;
	float b = lighting.fog_falloff;
	// view parameters
	float c = view.cam_pos.z;
	float d = ray_cam.z;
	
	// fog at height z is defined as F(z) = a*exp(-b*z)
	//  this creates fog density a at z=0 and exponential fallow parameterized by b
	// height along camera ray is Z(t) = c + d*t
	// then the fog is integrated along the ray with F(Z(t))
	// optical depth -> total "amount" of fog encountered
	
	float bd = b * d;
	const float eps = 1e-6;
	
	float fog_amount = a * exp(-b * c);
	if (abs(bd) > eps) {
		fog_amount *= (1.0 - exp(-bd * dist)) / bd;
		
		fog_amount = (a / bd) * (exp(-b * c) - exp(-b * (c + d * dist)));
	}
	else {
		// needed to avoid div by zero, apparently it's impossible to avoid this division with this formula
		// the only way woulb be to turn exp into it's series around 0, allowing you to approximate
		//  (1.0 - exp(-Tx))/T  as a whole, which gets rid of the division
		float approx = dist;
		approx -= bd * (dist*dist) * (1.0 / 2.0);
		//approx += bd*bd * (dist*dist*dist) * (1.0 / 6.0);
		//approx -= bd*bd*bd * (dist*dist*dist*dist) * (1.0 / 24.0);
		
		fog_amount *= approx;
	}
	
	// get transmittance (% of rays scattered/absorbed) from optical depth
	// -> missing in iquilezles's code?
	// I believe if at x fog thickness y% of rays are blocked, then the remaining light will (perhaps unintuitively) follow exponential falloff, resulting in this additional exp
	float t = exp(-fog_amount);
	
	// adjust color to give sun tint like iquilezles
	float sun_amount = max(dot(ray_cam, -lighting.sun_dir), 0.0);
	//vec3  col = mix(lighting.fog_col, lighting.sun_col, pow(sun_amount, 8.0) * 0.5);
	vec3 sun = lighting.sun_col - atmos_scattering();
	
	vec3 col = mix(lighting.fog_col, sun, pow(sun_amount, 8.0) * 0.7);
	
	//return vec3(1.0 - t);
	
	// lerp pixel color to fog color
	return mix(col * stren, pix_col, t);
}

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

vec3 sun_lighting (vec3 normal, float shadow) {
	float stren = sun_strength();
	
	vec3 sun = lighting.sun_col - atmos_scattering();
	
	float d = max(dot(-lighting.sun_dir, normal), 0.0);
	vec3 diffuse =
		(stren        ) * sun * vec3(d * shadow) +
		(stren + 0.008) * lighting.sky_col*0.18;
	
	return vec3(diffuse);
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
