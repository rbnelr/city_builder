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
		vec2 p = vec2(gl_VertexID & 1, (gl_VertexID >> 1) & 1);
		
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

	void main () {
		for (int face=0; face<6; ++face) {
			CubemapFace f = _cubemap_faces[face];
			for (int i=0; i<3; ++i) {
				gs_dir_world = f.world_dir +
				               f.world_u * (2.0*v_uv[i].x - 1.0) +
				               f.world_v * (2.0*v_uv[i].y - 1.0);

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
	
	uniform int mip_i;
	uniform float roughness;
	
	out vec3 frag_col;
	void main () {
		vec3 dir_world = normalize(gs_dir_world);
		vec3 ref_point = view.cam_pos;

		if (mip_i == 0) {
			// prefilter_env_map is equivalent to procedural_sky at roughness = 0
			// skip integration entirely, later mip passes can also access this which should be faster
			frag_col = procedural_sky(ref_point, dir_world);
		}
		else {
			// Optimize performance by reducing samples in low-roughness versions where sample count seems to be less important
			// samples are tightly clustered around the normal anyway
			// though there should be rare samples in other directions, but these are less important
			// But we really can't afford 4k samples at the higher resolutions in real time
			// and this code is supposed to run every frame to bake current time of day into the env map
			int num_samples = 512;
			if (roughness > 0.3) num_samples = 1024;
			if (roughness > 0.75) num_samples = 1024*4;

			frag_col = prefilter_env_map(roughness, ref_point, dir_world, num_samples);
		}
	}
#endif
