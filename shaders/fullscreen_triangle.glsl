
VS2FS Vertex {
	vec2 uv;
} v;

#ifdef _VERTEX
	void main () {
		// 2
		// | \
		// |  \
		// 0---1
		vec2 p = vec2(gl_VertexID & 1, (gl_VertexID >> 1) & 1);
		
		// triangle covers [-1, 3]
		// such that the result is a quad that fully covers [-1,+1]
		gl_Position = vec4(p * 4.0 - 1.0, 0.0, 1.0);
		v.uv        = vec2(p * 2.0);
	}
#endif
