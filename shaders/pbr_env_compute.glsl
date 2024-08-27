#version 430
#include "common.glsl"
#include "pbr.glsl"

#if MODE != 3
	// usually 8x8 pixels x 1 cubemap face in workgroup
	layout(local_size_x=8, local_size_y=8, local_size_z=1) in;
#else
	#define NUM_BATCHES 32
	#define GROUP_PIXELS 4
	
	layout(local_size_x=NUM_BATCHES, local_size_y=GROUP_PIXELS, local_size_z=1) in;
#endif

uniform ivec2 resolution;

vec3 _get_world_dir (ivec2 xy, int face) {
	vec2 uv = (vec2(xy)+0.5) / resolution;
	
	CubemapFace f = _cubemap_faces[face];
	return normalize(f.world_dir +
	                 f.world_u * (2.0*uv.x - 1.0) +
	                 f.world_v * (2.0*uv.y - 1.0));
}
vec3 get_world_dir () {
	ivec2 xy = ivec2(gl_GlobalInvocationID.xy);
	int face = int(gl_GlobalInvocationID.z);
	return _get_world_dir(xy, face);
}

#if MODE == 1
// Generate mipmaps
	uniform layout(binding=0, ENV_PIXEL_FORMAT) restrict writeonly imageCube tex_out;
	uniform samplerCube env_map;
	uniform float prev_lod;
	
	void main () {
		if (gl_GlobalInvocationID.x >= resolution.x || gl_GlobalInvocationID.y >= resolution.y) return;
		
		// Mipmap gen is simple, resulting texel color is bilinear sampled texels of previous lod
		// for exact /2 texture, this results in exactly 4 texels being avered exactly
		vec3 col = readCubemapLod(env_map, get_world_dir(), prev_lod).rgb;
		imageStore(tex_out, ivec3(gl_GlobalInvocationID), vec4(col, 1.0));
	}
#elif MODE == 2
// Copy mip0, TODO: optimize this away
	uniform layout(binding=0, ENV_PIXEL_FORMAT) restrict writeonly imageCube tex_out;
	uniform layout(binding=1, ENV_PIXEL_FORMAT) restrict readonly  imageCube tex_in;
	
	void main () {
		if (gl_GlobalInvocationID.x >= resolution.x || gl_GlobalInvocationID.y >= resolution.y) return;
		
		vec3 col = imageLoad(tex_in, ivec3(gl_GlobalInvocationID)).rgb;
		imageStore(tex_out, ivec3(gl_GlobalInvocationID), vec4(col, 1.0));
	}
#elif MODE == 3
	// 128-512 have smilght speckling even with the mipmap sampling, but only around the sun if the sun is way brighter than the sky around it
	// With the sun having a brightness of 100 it looks mostly fine with 512
	uniform int num_samples = 512;

	vec3 convolve_env_map (samplerCube base_env_map, float roughness, vec3 refl,
			int first_sample, int batch_samples) {
		// just use incoming light vector as both normal and view, which is not accurate
		vec3 normal = refl;
		vec3 view = refl;
		
		mat3 TBN = dodgy_TBN(normal);
		
		vec3 light_sum = vec3(0);
		float total_weight = 0;
		
		for (int i=first_sample; i < first_sample + batch_samples; i++) {
			vec2 sampl = hammersley(i, num_samples);
			vec3 half_dir = TBN * importance_sample_GGX(sampl, roughness);
			vec3 light_dir = reflect(-view, half_dir);
			
			float dotLN = dot(light_dir, normal);
			float dotHN = dot(half_dir, normal);
			float dotVH = dot(view, half_dir);
			if (dotLN > 0.0) {
			#if 0
				light_sum += readCubemapLod(base_env_map, light_dir, 0.0).rgb * dotLN;
			#else
				// From https://learnopengl.com/PBR/IBL/Specular-IBL
				// Sample mipmap instead of high-res env map, to eliminate light speckling artefact
				// This should allow lower sample count and boost filtering speed even more by improving caching at the same time
				float D   = distribution_GGX(roughness, dotHN);
				float pdf = D * dotHN / (4.0 * dotVH) + 0.0001;

				float resolution = float(textureSize(base_env_map, 0).x);
				float saTexel  = 4.0 * PI / (6.0 * resolution * resolution);
				float saSample = 1.0 / (float(num_samples) * pdf + 0.0001);

				float mipLevel = roughness == 0.0 ? 0.0 : 0.5 * log2(saSample / saTexel); 
				
				light_sum += readCubemapLod(base_env_map, light_dir, mipLevel).rgb * dotLN;
			#endif
				// weight light by dotLN, (but then divide it out of the sum in the end)
				// I don't quite understand this, but like the paper says, it looks more correct
				total_weight += dotLN;
			}
		}
		
		return light_sum / (total_weight * float(NUM_BATCHES));
	}

	uniform samplerCube base_env_map;
	uniform float roughness;
	uniform layout(binding=0, ENV_PIXEL_FORMAT) restrict writeonly imageCube tex_out;
	
	shared vec3 colors[GROUP_PIXELS][NUM_BATCHES];
	
	void main () {
		int sample_idx      = int(gl_LocalInvocationID.x);
		int local_pixel_idx = int(gl_LocalInvocationID.y);
		
		ivec2 xy = ivec2(gl_GlobalInvocationID.y % resolution.x,
		                 gl_GlobalInvocationID.y / resolution.x);
		int face = int(gl_GlobalInvocationID.z);
		if (xy.x >= resolution.x || xy.y >= resolution.y) return;
		
		vec3 dir_world = _get_world_dir(xy, face);
		
		int samples_per_batch = num_samples / NUM_BATCHES;
		vec3 col = convolve_env_map(base_env_map, roughness, dir_world,
			sample_idx*samples_per_batch, samples_per_batch);
		colors[local_pixel_idx][sample_idx] = col;
		
		if (sample_idx != 0) return;
		barrier();
		
		vec3 total = vec3(0.0);
		for (int i=0; i<NUM_BATCHES; ++i) {
			total += colors[local_pixel_idx][i];
		}
		
		//col = readCubemapLod(base_env_map, dir_world, 0.0).xyz;
		imageStore(tex_out, ivec3(xy, face), vec4(total, 1.0));
	}
#endif