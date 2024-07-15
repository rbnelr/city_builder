#version 430
#include "common.glsl"
#include "fullscreen_triangle.glsl"

#ifdef _FRAGMENT
	#include "pbr.glsl"
	
	uniform vec2 resolution;
	uniform int face_i;
	
	out vec4 frag_col;
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
		
		vec3 col = procedural_sky(ref_point, dir_world);
		col = apply_fog(col, ref_point + dir_world * 1000000.0);
		
		frag_col = vec4(col, 1.0);
	}
#endif
