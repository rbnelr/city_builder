#version 430
#include "common.glsl"
#include "fullscreen_triangle.glsl"

#ifdef _FRAGMENT
	#include "pbr.glsl"
	
	uniform vec2 resolution;
	uniform int face_i;
	uniform int mip_i;
	uniform float roughness;
	
	out vec3 frag_col;
	void main () {
		// Cubemap code
		vec2 uv = gl_FragCoord.xy / resolution;
		
		CubemapFace face = _cubemap_faces[face_i];
		vec3 dir_world = normalize(
			face.world_dir +
			face.world_u*(2.0*uv.x - 1.0) +
			face.world_v*(2.0*uv.y - 1.0)
		);
		//vec2 test = uv * 15;
		//if (int(test.y) == 1 && int(test.x) % 2 == 0 && int(test.x)/2 <= _face) {
		//	frag_col = vec4(1,0,0,1);
		//}
		
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
