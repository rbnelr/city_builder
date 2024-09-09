
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

#ifdef _FRAGMENT
#define GBUF_OUT \
	layout(location = 0) out vec4 frag_col; \
	layout(location = 1) out vec4 frag_emiss; \
	layout(location = 2) out vec4 frag_norm; \
	layout(location = 3) out vec4 frag_pbr;
	
uniform bool wireframe = false;
uniform vec4 wireframe_col = vec4(0,0,0,1);

#define GBUF_HANDLE_WIREFRAME \
	if (wireframe) { \
		frag_col   = vec4(0,0,0,wireframe_col.a); \
		frag_emiss = wireframe_col; \
		frag_norm  = vec4(0,0,1,wireframe_col.a); \
		frag_pbr   = vec4(1,0,0,wireframe_col.a); \
		return; \
	}
#endif
	
struct GbufResult {
	vec3  view_dir; // camera to point, or just pixel ray dir if !valid
	vec3  pos_world;
	vec3  normal_world;
	vec3  albedo;
	vec3  emissive ;
	float roughness;
	float metallic;
};

#ifdef _FRAGMENT
vec3 get_fragment_ray () {
	vec2 uv = gl_FragCoord.xy * view.inv_viewport_size;
	return normalize(depth_to_pos_world(0.5, uv) - view.cam_pos);
}
#endif

#if GBUF_IN
uniform sampler2D gbuf_depth;
uniform sampler2D gbuf_col;
uniform sampler2D gbuf_emiss;
uniform sampler2D gbuf_norm;
uniform sampler2D gbuf_pbr;

// decode only depth (for decal drawing)
bool decode_gbuf_pos (out vec3 pos_world) {
	// v.uv from fullscreen_triangle.glsl
	float depth = texelFetch(gbuf_depth, ivec2(gl_FragCoord.xy), 0).r;
	
	if (depth > 0.0) {
		vec2 uv = gl_FragCoord.xy * view.inv_viewport_size;
		
		pos_world = depth_to_pos_world(depth, uv);
		return true;
	}
	return false;
}
// full gbuffer decode for defferred lighting
bool decode_gbuf (out GbufResult r) {
	float depth = texelFetch(gbuf_depth, ivec2(gl_FragCoord.xy), 0).r;
	r.albedo    = texelFetch(gbuf_col,   ivec2(gl_FragCoord.xy), 0).rgb;
	r.emissive  = texelFetch(gbuf_emiss, ivec2(gl_FragCoord.xy), 0).rgb * lighting.inv_exposure;
	vec3 normal = texelFetch(gbuf_norm,  ivec2(gl_FragCoord.xy), 0).rgb;
	vec2 pbr    = texelFetch(gbuf_pbr,   ivec2(gl_FragCoord.xy), 0).rg;
	r.roughness = pbr.r;
	r.metallic  = pbr.g;
	
	vec2 uv = gl_FragCoord.xy * view.inv_viewport_size;
	
	// TODO: find a better way of doing this, including finding pos_world
	// probably by just getting a vector per pixel that, multiplied by a depth value, gives the position, but with reverse depth I might have to first remap reverse depth to 'real' depth
	vec3 fake_pos_world = depth_to_pos_world(0.01, uv);
	r.view_dir = normalize(fake_pos_world - view.cam_pos);
	if (depth > 0.0) {
		r.pos_world = depth_to_pos_world(depth, uv);
		r.normal_world = normalize(normal);
		return true;
	}
	return false;
}
#endif
