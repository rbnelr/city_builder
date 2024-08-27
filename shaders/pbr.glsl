
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
	float a = roughness*roughness;
	float a2 = a*a;
	
	float denom = dotHN*dotHN * (a2 - 1.0) + 1.0;
	return a2 / (PI * (denom*denom) + 0.000000000000000000000000001); // fixes nan/inf pixel in very center of highlight?
}

// Fixed F0 for non-metals like in epic paper
// In reality this is ((IOR_in - IOR_out) / (IOR_in + IOR_out))^2, ie. glass, water, plastic each reflect slight different amounts
// In practice this barely makes a difference (according to epic)
const float DIELECTRIC_F0 = 0.04; // ~matches glass (IOR_glass = 1.52) -> (-0.5/2.5)^2=0.04

vec3 fresnel_schlick (vec3 albedo, float metallic, float dotVH) {
	vec3 F0 = mix(vec3(DIELECTRIC_F0), albedo, metallic);
	
	// same as F0 + (1-F0)*(1.0 - dotVH)^5, but uses slightly less ops due to vec3 vs float
	float Fc = pow(1.0 - dotVH, 5.0);
	return F0 * (1.0 - Fc) + Fc;
}
vec3 fresnel_schlick_fast (vec3 F0, float dotVH) {
	// TODO: is this even faster than pow(1.0 - dotVH, 5.0)?
	float x = 1.0 - dotVH;
	float x2 = x*x;
	float Fc = x2*x2*x;
	return F0 * (1.0 - Fc) + Fc;
}
vec3 fresnel_schlick_approx (vec3 albedo, float metallic, float dotVH) {
	vec3 F0 = mix(vec3(DIELECTRIC_F0), albedo, metallic);
	
	float exp = (-5.55473 * dotVH - 6.98316) * dotVH;
	float Fc = pow(2.0, exp);
	return F0 * (1.0 - Fc) + Fc;
}

// Differing G term for analytical and IBL light, I don't understand this, epic paper said this is better

float geometry_schlick_analytical (float roughness, float dotVN, float dotLN) {
	// remap roughness: (r+1)/2, then a=r^2, then k = a/2
	// -> (r+1)^2 / 8
	float r = roughness + 1.0;
	float k = (r*r) * 0.125;
	
	float a = dotVN / (dotVN * (1.0-k) + k + 0.000001);
	float b = dotLN / (dotLN * (1.0-k) + k + 0.000001);
	return a * b;
}
float geometry_schlick_IBL (float roughness, float dotVN, float dotLN) {
	// a=r^2, then k = a/2
	// -> r^2 / 2
	float k = (roughness*roughness) * 0.5;
	
	float a = dotVN / (dotVN * (1.0-k) + k + 0.000001);
	float b = dotLN / (dotLN * (1.0-k) + k + 0.000001);
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

// These are for the split sum optimization:
// Instead of integrating the hemisphere of incoming light and applying the brdf for each light direction
// we integrate the hemisphere of incoming light, only weight it by cos theta and sum it up (independent of material)
// then seperately integrate the hemisphere of brdf for the material without knowing the light
// then later this is multiplied, and in practice looks extremely similar

// This is the accurate way of integrating the incoming light, where the light and view direction are known,
// but in practice we don't use this, because the power of the split sum opt ist that we can precompute both parts into textures, this particular texture ends up as a cubemap, which can only take in one direction, not two
vec3 convolve_env_map_accurate (float roughness, vec3 ref_point, vec3 view, vec3 normal, int num_samples) {
	mat3 TBN = dodgy_TBN(normal);
	
	vec3 light_sum = vec3(0);
	float total_weight = 0;
	
	for (int i=0; i<num_samples; i++) {
		vec2 sampl = hammersley(i, num_samples);
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
// This is the inaccurate version, which unfortunately introduces a noticable visual difference, in that shallow angle reflections are not streched, which looks noticably wrong, BUT in practice we only use this optimization or IBL, ie enviroment lighting, and add analytical light sources on top of this, which do not need integration at all, so specular highlights will usually still be accurate
vec3 convolve_env_map_procedural (float roughness, vec3 ref_point, vec3 refl, int num_samples) {
	// just use incoming light vector as both normal and view, which is not accurate
	vec3 normal = refl;
	vec3 view = refl;
	
	mat3 TBN = dodgy_TBN(normal);
	
	vec3 light_sum = vec3(0);
	float total_weight = 0;
	
	for (int i=0; i<num_samples; i++) {
		vec2 sampl = hammersley(i, num_samples);
		vec3 half_dir = TBN * importance_sample_GGX(sampl, roughness);
		vec3 light_dir = reflect(-view, half_dir);
		
		float dotLN = dot(light_dir, normal);
		if (dotLN > 0.0) {
			// weight light by dotLN, (but then divide it out of the sum in the end)
			// I don't quite understand this, but like the paper says, it looks more correct
			light_sum += procedural_sky(ref_point, light_dir) * dotLN;
			total_weight += dotLN;
		}
	}
	
	return light_sum / total_weight;
}
// This is the inaccurate version, which unfortunately introduces a noticable visual difference, in that shallow angle reflections are not streched, which looks noticably wrong, BUT in practice we only use this optimization or IBL, ie enviroment lighting, and add analytical light sources on top of this, which do not need integration at all, so specular highlights will usually still be accurate
vec3 convolve_env_map (samplerCube base_env_map, float roughness, vec3 refl, int num_samples) {
	// just use incoming light vector as both normal and view, which is not accurate
	vec3 normal = refl;
	vec3 view = refl;
	
	mat3 TBN = dodgy_TBN(normal);
	
	vec3 light_sum = vec3(0);
	float total_weight = 0;
	
	for (int i=0; i<num_samples; i++) {
		vec2 sampl = hammersley(i, num_samples);
		vec3 half_dir = TBN * importance_sample_GGX(sampl, roughness);
		vec3 light_dir = reflect(-view, half_dir);
		
		float dotLN = dot(light_dir, normal);
		if (dotLN > 0.0) {
			light_sum += readCubemapLod(base_env_map, light_dir, 0.0).rgb * dotLN;
			
			// weight light by dotLN, (but then divide it out of the sum in the end)
			// I don't quite understand this, but like the paper says, it looks more correct
			total_weight += dotLN;
		}
	}
	
	return light_sum / total_weight;
}

// Here we integrate the brdf according to just two float (unlike view and light direction + material props)
// This works by splitting the integral into two halves which allows to factor out F0
// (F0 was what albedo and metallic was needed for)
// In practice this allows us to turn this integral into a LUT texture with just x,y input and r,g output all in [0,1] range
// 256-512 resolution is enough
vec2 integrate_brdf (float dotVN, float roughness, int num_samples) {
	// F0 factored out of integral (turns into two 1d integral sums) -> brdf.x * F0 + brdf.y;
	// all calculations moved to tangent space (which is equivalent) -> normal = vec3(0,0,1)
	// brdf is symmetric in rotation around normal, ie importance_sample_GGX produces just as many -x as +x or +y etc.
	// so view_tang.xy can actually be inferred from view_tang.z alone (losing z axis rotation which does not matter)
	// -> eliminates view_tang.y, could allow for more explicit optimizations
	
	// Not needed for brdf LUT preintegration, (since no texel has 0 uvs)
	// but still do this for more accurate comparisons
	dotVN = max(dotVN, 0.000001);
	
	// view vector in tangent space inferred from dotVN
	vec3 view_tang = vec3(
		sqrt(1.0 - dotVN*dotVN),
		0,
		dotVN);
	
	vec2 brdf = vec2(0);
	for (int i=0; i<num_samples; i++) {
		vec2 sampl = hammersley(i, num_samples);
		vec3 half_dir = importance_sample_GGX(sampl, roughness);
		
		float light_dir_z = 2.0 * dot(view_tang, half_dir) * half_dir.z - view_tang.z;
		
		float dotLN = max(light_dir_z, 0.0);
		//float dotVN = max(view_tang.z, 0.0);
		float dotHN = max( half_dir.z, 0.0);
		
		// dot here never actually uses half_dir.y, could cut some math from importance_sample_GGX, but compiler probably does that already
		//float dotVH = max(dot(view_tang, half_dir), 0.0);
		float dotVH = max(view_tang.x * half_dir.x + view_tang.z * half_dir.z, 0.0);
		
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
	
	brdf *= 1.0 / float(num_samples);
	return brdf;
}

#if PBR_RENDER
uniform sampler2D   pbr_brdf_LUT;
uniform samplerCube pbr_env_map;
uniform float       pbr_env_map_last_mip;
uniform float       pbr_env_roughness_curve;

vec3 pbr_sample_specular_env_map (vec3 direction, float roughness) {
	float lod = pow(roughness, pbr_env_roughness_curve) * pbr_env_map_last_mip;
	return readCubemapLod(pbr_env_map, direction, lod).rgb * lighting.inv_exposure; // un-exposure correct (work in lux space)
}
vec3 pbr_sample_diffuse_env_map (vec3 direction) {
	return readCubemapLod(pbr_env_map, direction, pbr_env_map_last_mip).rgb * lighting.inv_exposure;
}
/*
// Accurate reference implementation for specular brdf for comparison, using one loop
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
			vec3 direct_light = readCubemapLod(pbr_env_map, light_dir, 0.0).rgb;
			//vec3 direct_light = procedural_sky(g.pos_world, light_dir);
			
			// compute geometric attenuation G, fresnel F
			// distribution D is implicit through importance sampling
			vec3 F0 = mix(vec3(DIELECTRIC_F0), g.albedo, g.metallic);
			vec3 F = fresnel_schlick_fast(F0, dotVH);
			float G = geometry_schlick_IBL(g.roughness, dotVN, dotLN); // geometry_schlick_IBL here looks way better?
			
			// no idea why this uses dotVH and is missing the 4, maybe implicit as well?
			// Full BRDF (*in_light) is the specular lighting (the reflected light) 
			vec3 brdf = F * (G * dotVH / (dotHN * dotVN));
			light += direct_light * brdf;
			
			// inverse of Fresnel is how much is absorbed, that modified by albedo (and pi) is diffuse light, which pulls light from all directions, but only for non-metals
			vec3 kDiff = mix(vec3(1) - F, vec3(0), vec3(g.metallic));
			vec3 diffuse = kDiff * g.albedo * (1.0 / PI);
			//light += ambient_col * diffuse; // TODO: fix fake diffuse env light
		}
	}
	
	return light / float(NUM_SAMPLES);
}

// Approximation for comparison, but not baked into textures, so just as slow still
// (split sum + using the math that can be baked into textures)
vec3 pbr_IBL_test (in GbufResult g) {
	vec3 refl = reflect(g.view_dir, g.normal_world);
	float dotVN = max(dot(-g.view_dir, g.normal_world), 0.0);

	vec3 F0 = mix(vec3(DIELECTRIC_F0), g.albedo, g.metallic);
	vec3 F = fresnel_schlick_fast(F0, dotVN);
	
	vec3 spec_light = convolve_env_map_procedural(g.roughness, g.pos_world, refl, 1024);
	//vec3 env_light = convolve_env_map_accurate(g.roughness, g.pos_world, -g.view_dir, g.normal_world, 1024);
	vec3 diff_light = convolve_env_map_procedural(1.0, g.pos_world, g.normal_world, 1024);
	
	vec2 brdf = integrate_brdf(dotVN, g.roughness, 1024);
	// evaluate final IBL
	vec3 specular = (brdf.x * F0 + brdf.y);
	vec3 diffuse = g.albedo * (1.0/PI) * (1.0 - F) * (1.0 - g.metallic);
	return spec_light * specular + diff_light * diffuse;
}*/

// Optimized specular approximation using textures, extremely fast
vec3 pbr_IBL (in GbufResult g) {
	vec3 refl = reflect(g.view_dir, g.normal_world);
	float dotVN = max(dot(-g.view_dir, g.normal_world), 0.0);

	// specular color from albedo and metallic
	// non-metals have F0 of DIELECTRIC_F0, metals have F0 of albedo
	// NOTE: non-metlas will use albedo for diffuse light, metals do not have diffuse
	vec3 F0 = mix(vec3(DIELECTRIC_F0), g.albedo, g.metallic);
	vec3 F = fresnel_schlick_fast(F0, dotVN);

	// Sample specular env cubemap (with baked in roughness D_GGX term) using reflection vector
	vec3 spec_light = pbr_sample_specular_env_map(refl, g.roughness).rgb;
	// Sample specular env cubemap with roughness=1, which is a simple cosine weighted distribution
	// using normal vector as diffuse IBL
	vec3 diff_light = pbr_sample_diffuse_env_map(g.normal_world).rgb;

	// Sample precomputed brdf LUT
	vec2 brdf = textureLod(pbr_brdf_LUT, vec2(dotVN, g.roughness), 0.0).rg;
	// evaluate final IBL
	vec3 specular = (brdf.x * F0 + brdf.y);
	vec3 diffuse = (1.0 - F) * (g.albedo * (1.0/PI) * (1.0 - g.metallic));
	return spec_light * specular + diff_light * diffuse;
	//return diff_light * diffuse;
	//return spec_light * specular;
}

// non-IBL, ie analytical brdf, which is resonably fast and does not need the optimizations above
// can be used for direction or point lights, simply multiply with effective light radiance after call (taking into account attenuation for point lights)
// This does not look all that great for glossy surfaces in practice due to lights being infinitely small
// TODO: add area light implemementation from epic paper
vec3 pbr_analytical_light (in GbufResult g, vec3 light_radiance, vec3 light_dir) {
	
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
	
	// compute distribution D, geometric attenuation G, fresnel F
	float D = distribution_GGX(g.roughness, dotHN);
	vec3  F = fresnel_schlick_approx(g.albedo, g.metallic, dotVH);
	float G = geometry_schlick_analytical(g.roughness, dotVN, dotLN);
	
	vec3 specular_brdf = F * (D * G / (4.0 * dotLN * dotVN + 0.000001));
	
	// inverse of Fresnel is how much is absorbed, that modified by albedo (and pi) is diffuse light, which pulls light from all directions
	vec3 diffuse = (1.0 - F) * (g.albedo * (1.0/PI) * (1.0 - g.metallic));
	
	return (specular_brdf + diffuse) * (light_radiance * dotLN);
	//return (diffuse) * (light_radiance * dotLN);
	//return (specular_brdf) * (light_radiance * dotLN);
}
#endif
