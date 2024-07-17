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
	out flat int gs_additive_layer_id;

	void main () {
		for (int face=0; face<6; ++face) {
			CubemapFace f = _cubemap_faces[face];
			for (int i=0; i<3; ++i) {
				gs_dir_world = f.world_dir +
				               f.world_u * (2.0*v_uv[i].x - 1.0) +
				               f.world_v * (2.0*v_uv[i].y - 1.0);
				gs_additive_layer_id = gl_PrimitiveIDIn;
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
	in flat int gs_additive_layer_id;
	
	out vec3 frag_col;
	
#if FIRST_MIP
	void main () {
		vec3 dir_world = normalize(gs_dir_world);
		vec3 ref_point = view.cam_pos;
		// prefilter_env_map is equivalent to procedural_sky at roughness = 0
		// skip integration entirely, later mip passes can also access this which should be faster
		frag_col = procedural_sky(ref_point, dir_world);
	}
#else
	uniform samplerCube base_env_map;
	
	uniform float roughness;
	uniform int additive_layers;
	
	vec3 prefilter_env_map_test (samplerCube base_env_map, float roughness, vec3 refl,
			int first_sample, int layer_samples, int num_samples) {
		// just use incoming light vector as both normal and view, which is not accurate
		vec3 normal = refl;
		vec3 view = refl;
		
		mat3 TBN = dodgy_TBN(normal);
		
		vec3 light_sum = vec3(0);
		float total_weight = 0;
		
		for (int i=first_sample; i < first_sample + layer_samples; i++) {
			vec2 sampl = hammersley(i, num_samples);
			vec3 half_dir = TBN * importance_sample_GGX(sampl, roughness);
			vec3 light_dir = reflect(-view, half_dir);
			
			float dotLN = dot(light_dir, normal);
			if (dotLN > 0.0) {
				// weight light by dotLN, (but then divide it out of the sum in the end)
				// I don't quite understand this, but like the paper says, it looks more correct
				light_sum += readCubemapLod(base_env_map, light_dir, 0.0).rgb * dotLN;
				total_weight += dotLN;
			}
		}
		
		return light_sum / (total_weight * float(additive_layers));
	}
	
	void main () {
		vec3 dir_world = normalize(gs_dir_world);
		
		// Optimize performance by reducing samples in low-roughness versions where sample count seems to be less important
		// samples are tightly clustered around the normal anyway
		// though there should be rare samples in other directions, but these are less important
		// But we really can't afford 4k samples at the higher resolutions in real time
		// and this code is supposed to run every frame to bake current time of day into the env map
		int num_samples = 512;
		if (roughness > 0.3) num_samples = 1024;
		if (roughness > 0.75) num_samples = 1024*4;
		
		int layer_samples = num_samples / additive_layers;
		int first_sample = layer_samples * gs_additive_layer_id;
		
		frag_col = prefilter_env_map_test(base_env_map, roughness, dir_world, first_sample, layer_samples, num_samples);
	}
#endif
#endif
