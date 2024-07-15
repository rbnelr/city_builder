
#include "procedural_sky.glsl"

//// PBR dev:
// based on Epic's Real Shading in Unreal Engine 4
// https://cdn2.unrealengine.com/Resources/files/2013SiggraphPresentationsNotes-26915738.pdf

float radical_inverse_VdC (uint bits) {
	#if 0
	bits = ( bits                << 16u) | ( bits                >> 16u);
	bits = ((bits & 0x00FF00FFu) <<  8u) | ((bits & 0xFF00FF00u) >>  8u);
	bits = ((bits & 0x0F0F0F0Fu) <<  4u) | ((bits & 0xF0F0F0F0u) >>  4u);
	bits = ((bits & 0x33333333u) <<  2u) | ((bits & 0xCCCCCCCCu) >>  2u);
	bits = ((bits & 0x55555555u) <<  1u) | ((bits & 0xAAAAAAAAu) >>  1u);
	#else
	// OpenGL >=4.00
	bits = bitfieldReverse(bits);
	#endif
	return float(bits) * 2.3283064365386963e-10; // 1.0 / 0x100000000
}
vec2 hammersley (int i, int N) {
	return vec2(float(i)/float(N), radical_inverse_VdC(uint(i)));
}

float distribution_GGX (float roughness, float dotHN) {
	float	a = roughness*roughness;
	float	a2 = a*a;
	
	float	denom = dotHN*dotHN * (a2 - 1.0) + 1.0;
	return a2 / (PI * (denom*denom) + 0.000000000000000000000000001); // fixes nan/inf pixel in very center of highlight?
}

const float DIELECTRIC_F0 = 0.04; // like in epic paper

vec3 fresnel_schlick (vec3 albedo, float metallic, float dotVH) {
	vec3 F0 = mix(vec3(DIELECTRIC_F0), albedo, metallic);
	
	// same as F0 + (1-F0)*(1.0 - dotVH)^5, but uses slightly less ops due to vec3 vs float
	float Fc = pow(1.0 - dotVH, 5.0);
	return F0 * (1.0 - Fc) + Fc;
}
vec3 fresnel_schlick_approx (vec3 albedo, float metallic, float dotVH) {
	vec3 F0 = mix(vec3(DIELECTRIC_F0), albedo, metallic);
	
	float exp = (-5.55473 * dotVH - 6.98316) * dotVH;
	float Fc = pow(2.0, exp);
	return F0 * (1.0 - Fc) + Fc;
}

float geometry_schlick_analytical (float roughness, float dotVN, float dotLN) {
	// remap roughness: (r+1)/2, then a=r^2, then k = a/2
	// -> (r+1)^2 / 8
	float r = roughness + 1.0;
	float k = (r*r) * 0.125;
	
	float denomA = dotVN * (1.0-k) + k + 0.000001;
	float denomB = dotLN * (1.0-k) + k + 0.000001;
	
	float a = dotVN / denomA;
	float b = dotLN / denomB;
	return a * b;
}
float geometry_schlick_IBL (float roughness, float dotVN, float dotLN) {
	// a=r^2, then k = a/2
	// -> r^2 / 2
	float k = (roughness*roughness) * 0.5;
	
	float denomA = dotVN * (1.0-k) + k + 0.000001;
	float denomB = dotLN * (1.0-k) + k + 0.000001;
	
	float a = dotVN / denomA;
	float b = dotLN / denomB;
	return a * b;
}

vec3 importance_sample_GGX (vec2 sampl, float roughness) {
	// roughness is remapped for some reason (to better distribute 'glossyness' over the parameter?)
	float a = roughness * roughness;
	
	// turn [0,1] sample point into importance sampled hemisphere direction
	// importance sampled here means that more samples point up than to the horizon of the hemisphere
	// this bias is actually based on the roughness, smooth surfaces have almost all point up, rough ones are more diversely spread
	// this also seems to be derived from the GGX D() normal distribution function, but I have no idea how to derive or prove this
	float phi = 2.0 * PI * sampl.x;
	float cos_theta = sqrt( (1.0 - sampl.y) / (1.0 + (a*a - 1.0) * sampl.y) );
	float sin_theta = sqrt( 1.0 - cos_theta * cos_theta );
	
	// this hemisphere direction actually represents a microfacet, ie a tiny mirror
	// rough surfaces are modelled as consisting of many microscopic tiny mirros with certain distributions
	
	// turn these values into actual vector
	// this vector is apparently what half, ie avg of light and view direction is
	// this makes sense, since in the microfacet model, only light reflected by microfacets is seen, ie the microfacets normal = half vector
	
	// this is in tangent space
	return vec3(sin_theta * cos(phi),
				sin_theta * sin(phi),
				cos_theta);
}

vec3 prefilter_env_map_accurate (float roughness, vec3 ref_point, vec3 view, vec3 normal) {
	const int NUM_SAMPLES = 1024;
	
	mat3 TBN = dodgy_TBN(normal);
	
	vec3 light_sum = vec3(0);
	float total_weight = 0;
	
	for (int i=0; i<NUM_SAMPLES; i++) {
		vec2 sampl = hammersley(i, NUM_SAMPLES);
		vec3 half_dir = TBN * importance_sample_GGX(sampl, roughness);
		vec3 light_dir = reflect(-view, half_dir);
		
		float dotLN = dot(light_dir, normal);
		if (dotLN > 0.0) {
			light_sum += procedural_sky(ref_point, light_dir) * dotLN;
			total_weight += dotLN;
		}
	}
	
	return light_sum / total_weight;
}
vec3 prefilter_env_map (float roughness, vec3 ref_point, vec3 refl) {
	const int NUM_SAMPLES = 1024;
	
	vec3 normal = refl;
	vec3 view = refl;
	
	mat3 TBN = dodgy_TBN(normal);
	
	vec3 light_sum = vec3(0);
	float total_weight = 0;
	
	for (int i=0; i<NUM_SAMPLES; i++) {
		vec2 sampl = hammersley(i, NUM_SAMPLES);
		vec3 half_dir = TBN * importance_sample_GGX(sampl, roughness);
		vec3 light_dir = reflect(-view, half_dir);
		
		float dotLN = dot(light_dir, normal);
		if (dotLN > 0.0) {
			light_sum += procedural_sky(ref_point, light_dir) * dotLN;
			total_weight += dotLN;
		}
	}
	
	return light_sum / total_weight;
}

vec2 integrate_brdf (float dotVN, float roughness) {
	const int NUM_SAMPLES = 1024;
	
	// F0 factored out of integral (turns into two 1d integral sums) -> brdf.x * F0 + brdf.y;
	// all calculations moved to tangent space (which is equivalent) -> normal = vec3(0,0,1)
	// brdf is rotationally symmetric, ie importance_sample_GGX produces just as many -x as +x or +y etc.
	// so view_tang.xy can actually be inferred from view_tang.z alone (losing z axis rotation which does not matter)
	// -> eliminates view_tang.y, could allow for more explicit optimizations
	
	//mat3 TBN = dodgy_TBN(normal);
	//mat3 iTBN = inverse(TBN);
	//
	//vec3 view_tang = iTBN * view;
	//vec3 norm_tang = vec3(0,0,1);
	
	// Not needed for brdf LUT preintegration, (since no texel has 0 uvs)
	// but still do this for more accurate comparisons
	dotVN = max(dotVN, 0.000001);
	
	//float dotVN = dot(view, normal);
	vec3 view_tang = vec3(
		sqrt(1.0 - dotVN*dotVN),
		0,
		dotVN);
	
	vec2 brdf = vec2(0);
	for (int i=0; i<NUM_SAMPLES; i++) {
		vec2 sampl = hammersley(i, NUM_SAMPLES);
		vec3 half_dir = importance_sample_GGX(sampl, roughness);
		//vec3 light_dir = reflect(-view_tang, half_dir);
		//
		//float dotLN = max(dot(light_dir, norm_tang), 0.005);
		//float dotVN = max(dot( view_tang, norm_tang), 0.005);
		//float dotHN = max(dot( half_dir, norm_tang), 0.005);
		//float dotVH = max(dot( view_tang,       half_dir), 0.005);
		
		float light_dir_z = 2.0 * dot(view_tang, half_dir) * half_dir.z - view_tang.z;
		
		float dotLN = max(light_dir_z, 0.0);
		//float dotVN = max(view_tang.z, 0.0);
		float dotHN = max( half_dir.z, 0.0);
		float dotVH = max(dot(view_tang, half_dir), 0.0);
		
		if (dotLN > 0.0) {
			
			float G = geometry_schlick_IBL(roughness, dotVN, dotLN);
			
			float tmp = G * dotVH / (dotHN * dotVN);
			
			// Fresnel
			//float Fc = pow(1.0 - dotVH, 5.0);
			float e = 1.0 - dotVH;
			float e2 = e*e;
			float Fc = e2*e2*e;
			
			brdf.x += tmp * (1.0 - Fc);
			brdf.y += tmp * Fc;
		}
	}
	
	brdf *= 1.0 / float(NUM_SAMPLES);
	return brdf;
}

#if PBR_RENDER
uniform sampler2D   pbr_brdf_LUT;
uniform samplerCube pbr_spec_env;

vec3 pbr_reference_env_light (in GbufResult g) {
	
	vec3 ambient_col = lighting.sky_col * (sun_strength() + 0.001);
	
	
	mat3 TBN = dodgy_TBN(g.normal_world);
	
	const int NUM_SAMPLES = 1024;
	
	vec3 light = vec3(0);
	for (int i=0; i<NUM_SAMPLES; i++) {
		
		// importance sample the hemisphere using hammersley
		vec2 sampl = hammersley(i, NUM_SAMPLES);
		// distributed according to roughness adjusted GGX D() function (I think)
		vec3 half_dir = TBN * importance_sample_GGX(sampl, g.roughness);
		// based on microfacet normal (half_dir), get reflection dir
		vec3 light_dir = reflect(g.view_dir, half_dir);
		
		// angle cosines clamped to avoid backfacing surfaces being black due to normal mapping
		// TODO: not sure if this is a good solution or not
		float dotLN = max(dot(  light_dir, g.normal_world), 0.000001);
		float dotVN = max(dot(-g.view_dir, g.normal_world), 0.005);
		float dotHN = max(dot(   half_dir, g.normal_world), 0.005);
		float dotVH = max(dot(-g.view_dir,       half_dir), 0.005);
		
		if (dotLN > 0.0) {
			// sample skybox, ie IBL, though currently I fake some scatter procedurally instead of a cubemap
			vec3 direct_light = readCubemap(pbr_spec_env, light_dir).rgb;
			//vec3 direct_light = procedural_sky(g.pos_world, light_dir);
			
			// compute geometric attenuation G, fresnel F
			// distribution D is implicit through importance sampling
			vec3 F = fresnel_schlick_approx(g.albedo, g.metallic, dotVH);
			float G = geometry_schlick_IBL(g.roughness, dotVN, dotLN); // geometry_schlick_IBL here looks way better?
			
			// no idea why this uses dotVH and is missing the 4, maybe implicit as well?
			// Full BRDF (*in_light) is the specular lighting (the reflected light) 
			vec3 brdf = F * (G * dotVH / (dotHN * dotVN));
			light += direct_light * brdf;
			
			// inverse of Fresnel is how much is absorbed, that modified by albedo (and pi) is diffuse light, which pulls light from all directions, but only for non-metals
			vec3 kDiff = mix(vec3(1) - F, vec3(0), vec3(g.metallic));
			vec3 diffuse = kDiff * g.albedo * (1.0 / PI);
			light += ambient_col * diffuse; // TODO: fix fake diffuse env light
		}
	}
	
	return light / float(NUM_SAMPLES);
}

vec3 pbr_approx_env_light_test (in GbufResult g) {
	//vec3 ambient_col = lighting.sky_col * (sun_strength() + 0.001);
	
	vec3 specular_color = mix(vec3(DIELECTRIC_F0), g.albedo, g.metallic);
	
	float dotVN = max(dot(-g.view_dir, g.normal_world), 0.0);
	vec3 refl = reflect(g.view_dir, g.normal_world);
	
	vec3 env_light = prefilter_env_map(g.roughness, g.pos_world, refl);
	//vec3 env_light = prefilter_env_map_accurate(g.roughness, g.pos_world, -g.view_dir, g.normal_world);
	
	vec2 brdf = integrate_brdf(dotVN, g.roughness);
	
	return env_light * (brdf.x * specular_color + brdf.y);
}
vec3 pbr_approx_env_light (in GbufResult g) {
	vec3 specular_color = mix(vec3(DIELECTRIC_F0), g.albedo, g.metallic);
	
	float dotVN = max(dot(-g.view_dir, g.normal_world), 0.0);
	vec3 refl = reflect(g.view_dir, g.normal_world);
	vec3 env_light = readCubemap(pbr_spec_env, refl).rgb;
	
	vec2 brdf = textureLod(pbr_brdf_LUT, vec2(dotVN, g.roughness), 0.0).rg;
	
	return env_light * (brdf.x * specular_color + brdf.y);
}

vec3 pbr_directional_light (in GbufResult g, vec3 light_radiance, vec3 light_dir) {
	
	vec3 half_dir = normalize(-g.view_dir + light_dir);
	
	// angle cosines clamped to avoid backfacing surfaces being black due to normal mapping
	// TODO: not sure if this is a good solution or not
	float dotLN =     dot(  light_dir, g.normal_world);
	float dotVN = max(dot(-g.view_dir, g.normal_world), 0.005);
	float dotHN = max(dot(   half_dir, g.normal_world), 0.005);
	float dotVH = max(dot(-g.view_dir,       half_dir), 0.005);
	
	if (dotLN <= 0.0) {
		return vec3(0);
	}
	
	// compute geometric attenuation G, fresnel F
	// distribution D is implicit through importance sampling
	float D = distribution_GGX(g.roughness, dotHN);
	vec3  F = fresnel_schlick_approx(g.albedo, g.metallic, dotVH);
	float G = geometry_schlick_analytical(g.roughness, dotVN, dotLN);
	
	// no idea why this uses dotVH and is missing the 4, maybe implicit as well?
	// Full BRDF (*in_light) is the specular lighting (the reflected light) 
	vec3 specular_brdf = F * (D * G / (4.0 * dotLN * dotVN + 0.000001));
	
	// inverse of Fresnel is how much is absorbed, that modified by albedo (and pi) is diffuse light, which pulls light from all directions
	vec3 kDiff = mix(vec3(1) - F, vec3(0), vec3(g.metallic));
	vec3 diffuse = kDiff * g.albedo * (1.0 / PI);
	
	//return vec3(G);
	
	return (specular_brdf + diffuse) * (light_radiance * dotLN);
}
#endif
