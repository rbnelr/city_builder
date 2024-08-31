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
	out vec3 frag_col;
	
	void main () {
		vec3 dir_world = normalize(gs_dir_world);
		vec3 ref_point = view.cam_pos;
		
		vec3 col = procedural_sky(ref_point, dir_world) * lighting.exposure;
		col = min(col, 2.0); // limit brightness because undersampling with bright sun causes instability, not sure if this breaks physical accuracy enough to matter
		frag_col = col;
	}
#endif
