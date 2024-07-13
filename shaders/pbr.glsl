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

vec3 importance_sample_GGX (vec2 sampl, float roughness, vec3 normal) {
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

vec3 pbr_reference_env_light (in GbufResult g, vec3 view_dir) {
	
	vec3 ambient_col = lighting.sky_col * (sun_strength() + 0.001);
	
	
	mat3 TBN = dodgy_TBN(g.normal_world);
	
	const int NUM_SAMPLES = 256;
	
	vec3 light = vec3(0);
	for (int i=0; i<NUM_SAMPLES; i++) {
		// importance sample the hemisphere using hammersley
		vec2 sampl = hammersley(i, NUM_SAMPLES);
		// distributed according to roughness adjusted GGX D() function (I think)
		vec3 half_dir = TBN * importance_sample_GGX(sampl, g.roughness, g.normal_world);
		// based on microfacet normal (half_dir), get reflection dir
		vec3 light_dir = reflect(-view_dir, half_dir);
		
		// angle cosines clamped to avoid backfacing surfaces being black due to normal mapping
		// TODO: not sure if this is a good solution or not
		float dotLN = max(dot(light_dir, g.normal_world), 0.005);
		float dotVN = max(dot( view_dir, g.normal_world), 0.005);
		float dotHN = max(dot( half_dir, g.normal_world), 0.005);
		float dotVH = max(dot( view_dir,       half_dir), 0.005);
		
		//if (dotLN > 0.0) {
			// sample skybox, ie IBL, though currently I fake some scatter procedurally instead of a cubemap
			vec3 direct_light = get_skybox_light(g.pos_world, light_dir);
			vec3 indirect_light = direct_light;
			
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
			
			//light += brdf;
		//}
	}
	
	return light / float(NUM_SAMPLES);
}

vec3 pbr_directional_light (in GbufResult g, vec3 view_dir,
		vec3 light_radiance, vec3 light_dir) {
	
	vec3 half_dir = normalize(view_dir + light_dir);
	
	// angle cosines clamped to avoid backfacing surfaces being black due to normal mapping
	// TODO: not sure if this is a good solution or not
	float dotLN =     dot(light_dir, g.normal_world);
	float dotVN = max(dot( view_dir, g.normal_world), 0.005);
	float dotHN = max(dot( half_dir, g.normal_world), 0.005);
	float dotVH = max(dot( view_dir,       half_dir), 0.005);
	
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

float specular_D_ggx (float dotHN, float roughness) {
	float a = roughness*roughness;
	float a2 = a*a;
	
	float denom = (dotHN*dotHN) * (a2 - 1.0) + 1.0;
	return a2 / (PI * (denom*denom));
}
float specular_G_schlick (float dotVN, float dotLN, float roughness) {
	float r = roughness + 1.0;
	float k = r * r * 0.125;
	
	float v = dotVN / (dotVN * (1.0-k) + k);
	float l = dotLN / (dotLN * (1.0-k) + k);
	return v * l;
}
float specular_F_schlick (float dotVH, float F0) {
	float ex = (-5.55473 * dotVH - 6.98316) * dotVH;
	return F0 + (1.0 - F0) * pow(2.0, ex);
}
