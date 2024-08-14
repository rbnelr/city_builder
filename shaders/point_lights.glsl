#version 430
#include "common.glsl"

VS2FS Vertex {
	// center of point light, NOT the fragment that was rendered, since that only covers those pixels possibly affected by the light!
	flat vec3   pos;
	flat float  radius;
	flat vec3   dir;
	flat vec2   cone;
	flat vec3   intensity; // in candela
} v;

#ifdef _VERTEX

layout(location = 0) in vec3   mesh_pos;
layout(location = 1) in vec3   inst_pos;
layout(location = 2) in float  inst_radius;
layout(location = 3) in vec3   inst_dir;
layout(location = 4) in vec2   inst_cone;
layout(location = 5) in vec3   inst_col;

void main () {
	vec3 world_pos = (mesh_pos * inst_radius) + inst_pos;
	
	v.pos       = inst_pos;
	v.radius    = inst_radius;
	v.dir       = normalize(inst_dir); // for good measure
	v.cone      = inst_cone;
	v.intensity = inst_col;
	
	if (distance(v.pos, view.cam_pos) < 30.0) {
		dbgdraw_point(v.pos, 0.5, vec4(1,1,0,1));
		dbgdraw_vector(v.pos, v.dir * 3.0f, vec4(1,0.8f,0,1));
	}
	
	gl_Position = view.world2clip * vec4(world_pos, 1.0);
}

#endif

#ifdef _FRAGMENT
	#define GBUF_IN 1
	#include "gbuf.glsl"
	#define PBR_RENDER 1
	#include "pbr.glsl"
	
	float falloff (float dist) {
		// quadratic falloff starting at 1 multiplied such that it reaches 2 at 
		//float x = dist / v.radius;
		//float atten = (1-x)/(x+1);
		//atten = clamp(atten, 0.0, 1.0);
		//return atten*atten;
		
		// Epic paper falloff
		float x = dist / v.radius;
		float x2 = x*x;
		float c = max(1.0 - x2*x2, 0.0);
		return (c*c) / (dist*dist + 1);
	}
	float cone_falloff (vec3 light2frag) {
		float t = map(dot(light2frag, v.dir), v.cone.y, v.cone.x);
		return smoothstep(0.0, 1.0, t);
		//return clamp(t, 0.0, 1.0);
	}
	
	out vec4 frag_col;
	void main () {
		GbufResult g;
		if (!decode_gbuf(g)) discard;
		
		vec3 frag2light = v.pos - g.pos_world;
		float dist_sqr = dot(frag2light, frag2light);
		if (dist_sqr > v.radius*v.radius)
			discard; // early out
		
		float dist = sqrt(dist_sqr);
		frag2light /= dist; // normalize
		
		float cone = cone_falloff(-frag2light);
		if (cone <= 0.00001)
			discard;
		
		float dotLN = max(dot(g.normal_world, frag2light), 0.0);
		vec3 light = cone * falloff(dist) * dotLN * v.intensity;
			
		vec3 col = pbr_analytical_light(g, light, frag2light);
		frag_col = vec4(col, 1.0);
		
		//frag_col = vec4(frag2light, 1.0);
		//frag_col = vec4(0.00005,0.0,0.0, 1.0);
		
		//frag_col = vec4(cone_falloff(-frag2light).xxx, 1.0);
	}
#endif
