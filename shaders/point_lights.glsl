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
	
	bool ray_cone_intersect (vec3 ray_pos, vec3 ray_dir, vec3 cone_pos, vec3 cone_dir, float cone_cos_half_ang,
			out vec2 t01) {
		vec3 D = ray_dir;
		vec3 V = cone_dir;
		vec3 CO = ray_pos - cone_pos;
		
		float D_V = dot(D, V);
		float CO_V = dot(CO, V);
		float cos2 = cone_cos_half_ang * cone_cos_half_ang;
		
		// t^2*a + t*b + c = 0
		float a = D_V*D_V - cos2;
		float b = 2.0 * (D_V*CO_V - dot(D, CO)*cos2);
		float c = CO_V*CO_V - dot(CO, CO)*cos2;
		
		// use pq formula
		if (a == 0.0) { // probably should be a <= epsilon
			// TODO: handle this case accurately!
			// t*b + c = 0  => t = -c/b
			// what if b == 0?   currently never actually triggers due to a being close to 0 but not equal
			//return false; // parallel ray to cone, math doesn't work!
		}
		
		float p = b / a * 0.5;
		float q = c / a;

		float root = p*p - q; // sqrt((p/2)^2 - q)
		if (root < 0.0)
			return false; // no solutions, ray misses both cones
		root = sqrt(root);
		float t0 = -root - p; // -p/2 +- sqrt((p/2)^2 - q)
		float t1 =  root - p;
		
		// check which hits hit real or fake (mirror) cone
		bool fake0 = dot(t0 * D + CO, V) < 0.0;
		bool fake1 = dot(t1 * D + CO, V) < 0.0;
		
		if (fake0 && fake1) // both hits on fake cone
			return false;
		
		if (fake0 || fake1) { // one hit on real cone, one fake
			if (fake0) { // ray towards real cone
				// exited fake cone, entered real cone(at least based on ray dir,
				//  ray might only start inside real and not actually touch fake, but this can later be determined by negative ts)
				t0 = t1; // t1 was actually the entry into real cone
				t1 = INF; // cannot exit out of cone again in this case (or there would be solution for t)
			}
			else { // ray towards fake cone
				// exited real cone, entered fake cone
				t1 = t0; // t0 was actually the exit from real cone
				t0 = -INF; // coming from inside real cone inf far
			}
		}
		// else: both hits on real cone
		
		t01 = vec2(t0, t1);
		return true;
	}
	
	float falloff (float dist) {
		// Epic paper light falloff
		float x = dist / light.radius;
		x *= x;
		x *= x; // ^4
		float fadeout = max(1.0 - x, 0.0);
		fadeout *= fadeout;
		
		float d = dist + 0.0;
		return fadeout / (d*d);
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
	
	float integrate_falloff (vec3 ray_pos, vec3 ray_dir, float ray_t0, float ray_t1) {
		float local_t = dot(light.pos - ray_pos, ray_dir); // distance along ray of closest point to light
		vec3 offset = ray_pos + ray_dir * local_t - light.pos; // offset from light at that point
		float h2 = dot(offset, offset); // squared distance to light
		
		// half of ray length through light volume sphere
		float t_half = sqrt(light.radius*light.radius - h2);
		
		// ray_start and fragment distance made relative to sphere
		float t0 = ray_t0 - local_t;
		float t1 = ray_t1 - local_t;
		// limit t to sphere for formula to work
		t0 = max(t0, -t_half);
		t1 = min(t1,  t_half);
		
		// ray starts behind light volume or geometry in front of light volume
		if (t0 >= t1)
			return 0.0;
		
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
	
		return res;
	}
	
	// scatter parts of the light towards to camera as if foggy
	vec3 airglow (bool gbuf_valid, vec3 frag_pos) {
		// ray based on pixel
		vec3 ray_start = view.cam_pos;
		vec3 ray_dir = get_fragment_ray();
		// distance along ray of gbuffer geometry (or infinitely far away for skybox)
		float frag_t = gbuf_valid ? dot(frag_pos - ray_start, ray_dir) : INF;
		
		// TODO: Make cones not be so sharp (respect inner&outer cone)
		// allow cone to be >180 degrees
		
		float middle_cone = (light.cone.x + light.cone.y) * 0.5;
		
		vec2 cone_t01;
		if (!ray_cone_intersect(ray_start, ray_dir, light.pos, light.dir, middle_cone, cone_t01))
			return vec3(0);
			
		float t0 = max(cone_t01.x, 0);
		float t1 = min(cone_t01.y, frag_t);
		if (t0 >= t1)
			return vec3(0);
		
		float total_light = integrate_falloff(ray_start, ray_dir, t0, t1);
		return total_light * 0.0001 * light.intensity;
	}
	
	out vec3 frag_col;
	void main () {
		GbufResult g;
		bool gbuf_valid = decode_gbuf(g);
		//if (!decode_gbuf(g)) discard;
		
		vec3 col = airglow(gbuf_valid, g.pos_world);
		
		vec3 frag2light = light.pos - g.pos_world;
		float dist_sqr = dot(frag2light, frag2light);
		if (dist_sqr < light.radius*light.radius) {
			float dist = sqrt(dist_sqr);
			frag2light /= dist; // normalize
			
			float cone = cone_falloff(-frag2light);
			if (cone > 0.0) {
				float dotLN = max(dot(g.normal_world, frag2light), 0.0);
				vec3 light = cone * dotLN * falloff(dist) * light.intensity;
				
				col += pbr_analytical_light(g, light, frag2light);
			}
		}
		
		frag_col = col * lighting.exposure; // exposure corrected
		
		//frag_col = frag2light;
		//frag_col = vec3(0.00005,0.0,0.0);
		
		//frag_col = cone_falloff(-frag2light).xxx;
	}
#endif
