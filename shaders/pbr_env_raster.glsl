#version 430
#include "common.glsl"

#ifdef _VERTEX
	layout(location = 0) out vec2 vs_uv;

	out vec2 v_uv;

	void main () {
		// 2
		// | \
		// |  \
		// 0---1
		int i = gl_VertexID % 3;
		vec2 p = vec2(i & 1, i >> 1);
		
		// triangle covers [-1, 3]
		// such that the result is a quad that fully covers [-1,+1]
		gl_Position = vec4(p * 4.0 - 1.0, 0.0, 1.0);
		v_uv        = vec2(p * 2.0);
	}
#endif
#ifdef _GEOMETRY
	layout(triangles) in;
	layout(triangle_strip, max_vertices = 18) out; // 3*6=18

	in vec2 v_uv[];
	out vec3 gs_dir_world;
	//out flat int gs_triangle_id;

	void main () {
		for (int face=0; face<6; ++face) {
			CubemapFace f = _cubemap_faces[face];
			for (int i=0; i<3; ++i) {
				gs_dir_world = f.world_dir +
				               f.world_u * (2.0*v_uv[i].x - 1.0) +
				               f.world_v * (2.0*v_uv[i].y - 1.0);
				//gs_triangle_id = gl_PrimitiveIDIn;
				gl_Layer = face;
				gl_Position = gl_in[i].gl_Position;
				EmitVertex();
			}
			EndPrimitive();
		}
	}
#endif
#ifdef _FRAGMENT
	#include "pbr.glsl"

	in vec3 gs_dir_world;
	//in flat int gs_additive_layer_id;
	
	out vec3 frag_col;
	
#if MODE == 0
	void main () {
		vec3 dir_world = normalize(gs_dir_world);
		vec3 ref_point = view.cam_pos;
		frag_col = procedural_sky(ref_point, dir_world) * lighting.exposure;
	}
#elif MODE == 1
	uniform samplerCube base_env_map;
	uniform float prev_lod;
	void main () {
		vec3 dir_world = normalize(gs_dir_world);
		frag_col = readCubemapLod(base_env_map, dir_world, prev_lod).rgb;
	}
#else
/*
	uniform samplerCube base_env_map;
	
	uniform float roughness;
	uniform int additive_layers;
	
	// 128-512 have smilght speckling even with the mipmap sampling, but only around the sun if the sun is way brighter than the sky around it
	// With the sun having a brightness of 100 it looks mostly fine with 512
	const int NUM_SAMPLES = 512;
	
	vec3 convolve_env_map_test (samplerCube base_env_map, float roughness, vec3 refl,
			int first_sample, int layer_samples) {
		// just use incoming light vector as both normal and view, which is not accurate
		vec3 normal = refl;
		vec3 view = refl;
		
		mat3 TBN = dodgy_TBN(normal);
		
		vec3 light_sum = vec3(0);
		float total_weight = 0;
		
		for (int i=first_sample; i < first_sample + layer_samples; i++) {
			vec2 sampl = hammersley(i, NUM_SAMPLES);
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
				float saSample = 1.0 / (float(NUM_SAMPLES) * pdf + 0.0001);

				float mipLevel = roughness == 0.0 ? 0.0 : 0.5 * log2(saSample / saTexel); 
				
				light_sum += readCubemapLod(base_env_map, light_dir, mipLevel).rgb * dotLN;
			#endif
				// weight light by dotLN, (but then divide it out of the sum in the end)
				// I don't quite understand this, but like the paper says, it looks more correct
				total_weight += dotLN;
			}
		}
		
		return light_sum / (total_weight * float(additive_layers));
	}
	
	void main () {
		vec3 dir_world = normalize(gs_dir_world);
		
		int layer_samples = NUM_SAMPLES / additive_layers;
		int first_sample = layer_samples * gs_additive_layer_id;
		frag_col = convolve_env_map_test(base_env_map, roughness, dir_world, first_sample, layer_samples);
	}
*/
#endif
#endif
