
uniform sampler2DArray       shadowmap;
uniform sampler2DArrayShadow shadowmap2;

uniform mat4  shadowmap_mat;
uniform vec3  shadowmap_dir; // light dir of sun
uniform float shadowmap_bias_fac = 2.0;
uniform float shadowmap_bias_max = 10.0;
uniform float shadowmap_cascade_factor;

float sun_shadowmap (vec3 pos_world, vec3 normal) {
	vec2 texelSize = 1.0 / textureSize(shadowmap, 0).xy;
	int cascades = int(textureSize(shadowmap, 0).z);
	
	float bias = shadowmap_bias_fac * tan(acos(dot(normal, -shadowmap_dir)));
	bias = clamp(bias, 0.0, shadowmap_bias_max);
	
	vec4 shadow_clip = shadowmap_mat * vec4(pos_world, 1.0);
	vec3 shadow_ndc = shadow_clip.xyz / shadow_clip.w;
	// [0,1] -> [-1,+1] because else the cascade logic becomes harder
	shadow_ndc.z = shadow_ndc.z*2.0 - 1.0;
	
	vec3 v = abs(shadow_ndc);
	float m = max(max(v.x,v.y), v.z);
	m = log(m) / log(shadowmap_cascade_factor);
	float cascade = max(ceil(m), 0.0);
	
	float scale = pow(shadowmap_cascade_factor, cascade);
	
	if (cascade >= cascades)
		return 1.0;
	
	shadow_ndc /= scale;
	
	vec2 shadow_uv = shadow_ndc.xy * 0.5 + 0.5;
	
	// TODO: fix this function wihout reverse_depth?
	// ndc is [-1,1] except for with z with reverse depth which is [0,1]
	// NOTE: not to be confused, shadow rendering uses ortho camera which uses a linear depth range unlike normal camera!
	// could use [-1,1] range here
	
	shadow_ndc.z = shadow_ndc.z*0.5 + 0.5; // [-1,+1] -> [0,1]
	if (shadow_ndc.z < 0.0)
		return 1.0;
	
	// TODO: need different bias for other cascades?
	float compare = shadow_ndc.z + bias;
	
	float shadow_fac = 0.0;
	
	// PCF
	int sum = 0;
	for (int x=-1; x<=1; ++x)
	for (int y=-1; y<=1; ++y) {
		vec2 uv = shadow_uv + vec2(x,y) * texelSize;
		
		float fac = texture(shadowmap2, vec4(uv, cascade, compare)).r;
		return fac;
		//float fac = compare > texture(shadowmap, vec3(uv, float(cascade))).r ? 1.0 : 0.0;
		shadow_fac += fac;
		sum++;
	}
	shadow_fac /= float(sum);
	
	return shadow_fac;
}
