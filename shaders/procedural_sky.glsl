uniform sampler2D clouds;
uniform samplerCube night_sky;
uniform sampler2D moon;
uniform sampler2D moon_nrm;

vec3 dir2sun () {
	return mat3(lighting.sun2world) * vec3(0,0,1);
}

vec3 nightSky_light (vec3 dir_world) {
	if (dir_world.z < -0.04)
		return vec3(0);
	vec3 dir = mat3(lighting.world2solar) * dir_world;
	return map(dir_world.z, -0.04, 0.0) * readCubemap(night_sky, dir).rgb * 0.001;
}

// sun gets darker towards horizon and to 0 at night
float sun_strength () {
	float a = map(dir2sun().z, 0.5, -0.1);
	return smoothstep(1.0, 0.0, a);
}
// removes some color of sun depending on distance to horizon
vec3 atmos_scattering (vec3 sun) {
	float a = map(dir2sun().z, 0.5, 0.0);
	return clamp(sun - vec3(0.0, 0.90, 0.95)*0.85 * smoothstep(0.0, 1.0, a), vec3(0), vec3(1));
}
// slightly darker horizon and "ground"
vec3 horizon (vec3 dir_world) {
	return vec3(clamp(map(dir_world.z, -1.0, 1.0), 0.5, 1.0));
}

// kinda dodgy way of drawing sun disc
vec3 draw_sun (vec3 sun_color, float sun_stren, vec3 dir_world) {
	sun_color *= sun_stren;

	float deg = acos(dot(dir_world, dir2sun()));
	
	const float sun_ang_size_half = 0.53*2.0 * (PI/180.0) * 0.5; // 0.53 degrees in real life!
	const float sun_falloff = sun_ang_size_half * 0.5;
	float c = 1.0 - clamp(map(deg, sun_ang_size_half, sun_ang_size_half + sun_falloff), 0.0, 1.0);
	c = c*c;

	return sun_color * 10000.0 * c; // sun light in 0-100 range
}

vec3 draw_moon (vec3 col, float sun_stren, vec3 dir_world) {
	const float moon_ang_size = 0.53*2.0 * (PI/180.0);
	const float moon_half_flat_size = tan(moon_ang_size*0.5)*2.0;
	
	vec3 dir = mat3(lighting.world2moon) * dir_world;
	if (dir.z < 0.0)
		return col;
	
	vec2 proj = dir.xy / dir.z;
	vec2 uv = (proj / moon_half_flat_size) + 0.5;
	if (uv.x > 0.0 && uv.x < 1.0 && uv.y > 0.0 && uv.y < 1.0) {
		// are these normals correct?
		vec3 nrm = pow(texture(moon_nrm, uv).rgb, vec3(1.0/2.2)).rgb * 2.0 - 1.0;
		vec4 albedo = texture(moon, uv);
		
		vec3 sun_dir_in_moon_space = (mat3(lighting.world2moon) * dir2sun()) * vec3(1,-1,-1);
		
		float sun_bright = max(dot(sun_dir_in_moon_space, nrm), 0.0) * 10.0;
		
		vec3 light = albedo.rgb * (sun_bright * lighting.sun_col * 0.05 + 0.00004); // TODO: find accurate value
		
		col = mix(col, light, albedo.aaa);
	}
	return col;
}

//
vec3 draw_clouds (vec3 col, float sun_stren, vec3 view_point, vec3 dir_world) {
	float t = (lighting.clouds_z - view_point.z) / dir_world.z;
	if (dir_world.z > 0.0 && t >= 0.0) {
		vec3 pos = view_point + t * dir_world;
		
		vec2 uv = pos.xy / lighting.clouds_sz + lighting.clouds_offset;
		uv.y = 1.0 - uv.y; // all textures are flipped!
		vec4 c = texture(clouds, uv);
		
		col = mix(col.rgb, c.rgb * sun_stren * 1.5, vec3(c.a * 0.9));
	}
	return col;
}

// analytical exponential height fog, fog fades incoming light to scattered sunlight color, or for color
vec3 apply_fog (vec3 pix_col, vec3 pix_pos) {
	// from https://iquilezles.org/articles/fog/
	// + my own research and derivation
	
	// TODO: properly compute extinction (out-scattering + absoption) and in-scattering
	// seperating in-scattering may allow for more blue tint at distance
	
	vec3 ray_cam = pix_pos - view.cam_pos;
	float dist = length(ray_cam);
	ray_cam = normalize(ray_cam);
	
	float stren = sun_strength();
	
	// exponential height fog parameters
	float a = lighting.fog_base;
	float b = lighting.fog_falloff;
	// view parameters
	float c = view.cam_pos.z;
	float d = ray_cam.z;
	
	// fog at height z is defined as F(z) = a*exp(-b*z)
	//  this creates fog density a at z=0 and exponential fallow parameterized by b
	// height along camera ray is Z(t) = c + d*t
	// then the fog is integrated along the ray with F(Z(t))
	// optical depth -> total "amount" of fog encountered
	
	float bd = b * d;
	const float eps = 1e-6;
	
	float fog_amount = a * exp(-b * c);
	if (abs(bd) > eps) {
		fog_amount *= (1.0 - exp(-bd * dist)) / bd;
		
		fog_amount = (a / bd) * (exp(-b * c) - exp(-b * (c + d * dist)));
	}
	else {
		// needed to avoid div by zero, apparently it's impossible to avoid this division with this formula
		// the only way woulb be to turn exp into it's series around 0, allowing you to approximate
		//  (1.0 - exp(-Tx))/T  as a whole, which gets rid of the division
		float approx = dist;
		approx -= bd * (dist*dist) * (1.0 / 2.0);
		//approx += bd*bd * (dist*dist*dist) * (1.0 / 6.0);
		//approx -= bd*bd*bd * (dist*dist*dist*dist) * (1.0 / 24.0);
		
		fog_amount *= approx;
	}
	
	// get transmittance (% of rays scattered/absorbed) from optical depth
	// -> missing in iquilezles's code?
	// I believe if at x fog thickness y% of rays are blocked, then the remaining light will (perhaps unintuitively) follow exponential falloff, resulting in this additional exp
	float t = exp(-fog_amount);
	
	// adjust color to give sun tint like iquilezles
	float sun_amount = max(dot(ray_cam, dir2sun()), 0.0);
	//vec3  col = mix(lighting.fog_col, lighting.sun_col, pow(sun_amount, 8.0) * 0.5);
	vec3 sun = atmos_scattering(lighting.sun_col) * 5;
	
	vec3 col = mix(lighting.fog_col, sun, pow(sun_amount, 7.0) * 0.7);
	
	//return vec3(1.0 - t);
	
	// lerp pixel color to fog color
	return mix(col * stren, pix_col, t);
}

vec3 procedural_sky (vec3 view_point, vec3 dir_world) {
	float stren = sun_strength() * 1.0;
	vec3 sun = atmos_scattering(lighting.sun_col);
	
	vec3 col = nightSky_light(dir_world);
	
	col += draw_sun(sun, stren, dir_world); // sun additive (TODO: should overwrite night sky)
	col = draw_moon(col, stren, dir_world); // moon overwrites sun
	
	// sky color over moon
	col += lighting.sky_col * stren;
	
	col *= horizon(dir_world);
	
	
	col = draw_clouds(col, stren, view_point, dir_world);
	
	// sublely green ground color
	vec3 ground_col = vec3(0.24, 0.26, 0.24) * stren;
	col = mix(col, ground_col, vec3(clamp(map(dir_world.z, 0.0, -0.01), 0.0, 1.0)));
	
	// fake ground plane to get fog kinda over ground at horizon
	float ground_z = -300;
	float fog_dist = 1000000.0;
	{
		float t = (ground_z - view_point.z) / dir_world.z;
		if (dir_world.z < 0.0 && t >= 0.0) {
			fog_dist = t;
		}
	}
	
	col = apply_fog(col, view_point + dir_world * fog_dist);
	return col;
} 

