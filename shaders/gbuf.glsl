
vec3 depth_to_pos_cam (float depth, vec2 screen_uv) {
	// TODO: debug why this does not work with non-reversed depth
	
	float a = view.cam2clip[2].z;
	float b = view.cam2clip[3].z;
	float c = view.cam2clip[2].w;
	float d = view.cam2clip[3].w;
	
	//  clip = cam2clip * vec4(*,*,cam_z,1)
	//  clip /= clip.w
	// => (cam_z*a + b) / (cam_z*c + d)
	// solved for cam_z
	float cam_z = (b - depth*d) / (depth*c - a);
	
	//float clip_z = a*cam_z + b;
	float clip_w = c*cam_z + d; // find clip_w
	
	vec2 screen_xy = screen_uv * 2.0 - 1.0;
	vec2 clip_xy = screen_xy * clip_w;// reverse perspective divide
	vec2 cam_xy = (view.clip2cam * vec4(clip_xy, 0.0, 1.0)).xy;
	
	return vec3(cam_xy, cam_z);
}
vec3 depth_to_pos_world (float depth, vec2 screen_uv) {
	vec3 pos_cam = depth_to_pos_cam(depth, screen_uv);
	vec4 pos_world = view.cam2world * vec4(pos_cam, 1.0);
	
	return pos_world.xyz;
}

#define GBUF_OUT \
	layout(location = 0) out vec4 frag_col; \
	layout(location = 1) out vec4 frag_norm;
	
struct GbufResult {
	bool valid;
	vec3 pos_world;
	vec3 norm_world;
	vec3 albedo;
};

#if GBUF_IN
uniform sampler2D gbuf_depth;
uniform sampler2D gbuf_col;
uniform sampler2D gbuf_norm;

bool decode_gbuf (out GbufResult r) {
	vec2 uv = gl_FragCoord.xy * view.inv_viewport_size;
	
	float depth = texture(gbuf_depth, uv).r;
	vec3 col    = texture(gbuf_col,   uv).rgb;
	vec3 normal = texture(gbuf_norm,  uv).rgb;
	
	r.albedo = col;
	
	if (depth > 0.0) {
		r.norm_world = normalize(normal);
		r.pos_world = depth_to_pos_world(depth, uv);
		return true;
	}
	return false;
}
#endif
