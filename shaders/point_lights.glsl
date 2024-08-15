#version 430
#include "common.glsl"

VS2FS Vertex {
	// center of point light, NOT the fragment that was rendered, since that only covers those pixels possibly affected by the light!
	flat vec3   pos;
	flat float  radius;
	flat vec3   dir;
	flat vec2   cone;
	flat vec3   intensity; // in candela
} light;

#ifdef _VERTEX

layout(location = 0) in vec3   mesh_pos;
layout(location = 1) in vec3   inst_pos;
layout(location = 2) in float  inst_radius;
layout(location = 3) in vec3   inst_dir;
layout(location = 4) in vec2   inst_cone;
layout(location = 5) in vec3   inst_col;

void main () {
	vec3 world_pos = (mesh_pos * inst_radius) + inst_pos;
	
	light.pos       = inst_pos;
	light.radius    = inst_radius;
	light.dir       = normalize(inst_dir); // for good measure
	light.cone      = inst_cone;
	light.intensity = inst_col;
	
	//if (distance(light.pos, view.cam_pos) < 30.0) {
	//	dbgdraw_point(light.pos, 0.5, vec4(1,1,0,1));
	//	dbgdraw_vector(light.pos, light.dir * 3.0f, vec4(1,0.8f,0,1));
	//}
	
	gl_Position = view.world2clip * vec4(world_pos, 1.0);
}

#endif

#ifdef _FRAGMENT
	#define GBUF_IN 1
	#include "gbuf.glsl"
	#define PBR_RENDER 1
	#include "pbr.glsl"
	
	float falloff (float dist) {
		// Epic paper light falloff
		float x = dist / light.radius;
		float x2 = x*x;
		float c = max(1.0 - x2*x2, 0.0);
		return (c*c) / (dist*dist + 1);
	}
	float cone_falloff (vec3 light2frag) {
		float inner_angle = light.cone.x;
		float outer_angle = light.cone.y;
		// fade out between cosine of inner and outer angle
		// TODO: does actually fading in real angles look any better? would need an acos
		float t = map(dot(light2frag, light.dir), outer_angle, inner_angle);
		//return smoothstep(0.0, 1.0, t);
		return clamp(t, 0.0, 1.0);
	}
	
	// scatter parts of the light towards to camera as if foggy
	vec3 airglow (bool gbuf_valid, vec3 frag_pos) {
		// ray based on pixel
		vec3 ray_start = view.cam_pos;
		vec3 ray_dir = get_fragment_ray();
		// distance along ray of gbuffer geometry (or infinitely far away for skybox)
		float frag_t = gbuf_valid ? dot(frag_pos - ray_start, ray_dir) : 999999999.0;
		
		float local_t = dot(light.pos - ray_start, ray_dir); // distance along ray of closest point to light
		vec3 offset = ray_start + ray_dir * local_t - light.pos; // offset from light at that point
		float h2 = dot(offset, offset); // squared distance to light
		
		// half of ray length through light volume sphere
		float t_half = sqrt(light.radius*light.radius - h2);
		
		// ray_start and fragment distance made relative to sphere
		float t0 =        - local_t;
		float t1 = frag_t - local_t;
		// limit t to sphere for formula to work
		t0 = max(t0, -t_half);
		t1 = min(t1,  t_half);
		
		// ray starts behind light volume or geometry in front of light volume
		if (t0 >= t1) return vec3(0.0);
		
		// light falloff (epic pbr paper version) integrated using wolfram alpha
		// using light falloff parameterized with h (closest ray distance) and r (light radius of the falloff)
		// integrated over t for the ray, formula only valid if t0 and t1 within r
		
		// some values for the formula
		float r2 = light.radius * light.radius;
		float r4 = r2 * r2;
		float b = 1.0 / sqrt(h2 + 1.0);
		
		// I don't think this can be simplified, unless you change the orginal falloff function to be better suited for integration?
		float res = (b*r4 - b) * (atan(b * t1) - atan(b * t0));
		res += (t1-t0) - h2*(t1-t0) + (t0*t0*t0 - t1*t1*t1)*0.333333333;
		res *= 1.0 / r4;
		res = max(res, 0.0);
		
		res *= res; // squared looks a bit more natural?
		return res * 0.0001 * light.intensity; // arbitrary 'in scatter' factor?
	}
	
	out vec3 frag_col;
	void main () {
		GbufResult g;
		bool gbuf_valid = decode_gbuf(g);
		//if (!decode_gbuf(g)) discard;
		
		frag_col = airglow(gbuf_valid, g.pos_world);
		
		vec3 frag2light = light.pos - g.pos_world;
		float dist_sqr = dot(frag2light, frag2light);
		if (dist_sqr >= light.radius*light.radius)
			return;
			
		float dist = sqrt(dist_sqr);
		frag2light /= dist; // normalize
		
		float cone = cone_falloff(-frag2light);
		if (cone <= 0.0)
			return;
		
		float dotLN = max(dot(g.normal_world, frag2light), 0.0);
		vec3 light = cone * dotLN * falloff(dist) * light.intensity;
		
		frag_col += pbr_analytical_light(g, light, frag2light);
		
		//frag_col = frag2light;
		//frag_col = vec3(0.00005,0.0,0.0);
		
		//frag_col = cone_falloff(-frag2light).xxx;
	}
#endif
