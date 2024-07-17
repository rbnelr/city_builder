#version 430
#include "common.glsl"

VS2FS Vertex {
	// center of point light, NOT the fragment that was rendered, since that only covers those pixels possibly affected by the light!
	flat vec3  light_pos;
	flat float light_radius;
	flat vec3  light_col;
} v;

#ifdef _VERTEX

layout(location = 0) in vec3  mesh_pos;
layout(location = 1) in vec3  instance_pos;
layout(location = 2) in float instance_radius;
layout(location = 3) in vec3  instance_col;

void main () {
	vec3 world_pos = (mesh_pos * instance_radius) + instance_pos;
	
	v.light_pos    = instance_pos;
	v.light_radius = instance_radius;
	v.light_col    = instance_col;
	
	gl_Position = view.world2clip * vec4(world_pos, 1.0);
}

#endif

#ifdef _FRAGMENT
	#define GBUF_IN 1
	#include "gbuf.glsl"
	#define PBR_RENDER 1
	#include "pbr.glsl"
	
	float attenuation (float dist) {
		// quadratic attenuation starting at 1 multiplied such that it reaches 2 at 
		float x = dist / v.light_radius;
		float atten = (1-x)/(x+1);
		atten = clamp(atten, 0.0, 1.0);
		return atten*atten;
	}
	
	out vec4 frag_col;
	void main () {
		GbufResult g;
		if (!decode_gbuf(g)) discard;
		
		vec3 frag2light = v.light_pos - g.pos_world;
		float dist_sqr = dot(frag2light, frag2light);
		if (dist_sqr > v.light_radius*v.light_radius)
			discard; // early out 
		
		float dist = sqrt(dist_sqr);
		frag2light /= dist; // normalize
		
		float atten = attenuation(dist);
		atten *= max(dot(g.normal_world, frag2light), 0.0);
		
		//vec3 light = g.albedo * v.light_col * atten;
		//frag_col = vec4(light, 1.0);
		
		{
			vec3 light = atten * v.light_col * 3.5; // TODO: fix light brightnesses (epic uses lumen)
			
			vec3 col = pbr_analytical_light(g, light, frag2light);
			frag_col = vec4(col, 1.0);
		}
		//frag_col = vec4(frag2light, 1.0);
		//frag_col = vec4(0.1,0.0,0.0, 1.0);
		
		//frag_col = vec4(0,0,0, 1.0);
	}
#endif
