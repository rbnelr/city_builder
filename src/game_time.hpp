#pragma once
#include "common.hpp"

// TODO: Hide exact day date (of month) and just do (early/middle/late July. etc) and then add a 7 day week cycle to the time of day sim
// This way we have coherent days (though can be less than 24h ingame), and actual weekends
// Hide day counter, and add weekday counter so that disconnect between days and years is less obvious
//  weekends could have almost no one going to work etc.
// can of course speed up time of day, but if this is so sped up that people arrive at work when they have to leave already this might be bad
//  but you could essentially speed up the time where most are at work, and most importantly the night
// this could be configurable via a menu, simply add a time_day_speed up to certain hours of the day at the cost of more traffic
// let user configure work-from-home percentage, or possibly even an entire trip-percentage
//  (avoid cities skylines thing where things just teleport, just make it so the same economics happen in half the trips)
// ...But ideally we support enough speedup, and model cities will naturally house less people than real life, so you don't get as much traffic
//   or just build kickass walkable cities and public transit

struct GameTime {
	// Don't save pause_sim because it's annying me that it's always paused on start
	SERIALIZE(GameTime, target_gamespeed, /*pause_sim,*/ time_of_day, day_duration, pause_day, time_of_year, year_duration, pause_year,
		planet_axial_tilt, map_latitude, map_rotation)
	
	static constexpr int   DAYS_IN_YEAR = 365; // do not simulate leap years
	static constexpr int   MOON_ORBITS_PER_YEAR = 13; // pretend moon orbits 13x a year and 
	static constexpr float MOON_ORBIT_INCLINATION = deg(5.14f); // orbit perfectly circular for simplicity
	static constexpr float DECEMBER_SOLSTICE = 10.4f / DAYS_IN_YEAR; // Offset to turn time_of_year into astronomical orbit for accurate seasons
	// NOTE: No timezones (longitude is assumed as 0) this means no winter/summer time! You might be suprized by the sunrise/sunset times

	struct MonthInfo {
		int first_day, days;
		const char* name;
	};
	static constexpr MonthInfo MONTHS[12] = {
		{   0, 31, "January" },
		{  31, 28, "February" },
		{  59, 31, "March" },
		{  90, 30, "April" },
		{ 120, 31, "May" },
		{ 151, 30, "June" },
		{ 181, 31, "July" },
		{ 212, 31, "August" },
		{ 243, 30, "September" },
		{ 273, 31, "October" },
		{ 304, 30, "November" },
		{ 334, 31, "December" }, // = 365
	};
	
	// target for gametime / real-life time, game might run slower if sim can't keep up (once I have a tick system)
	// day/year should be incremented by the acually simulated time instead of the target
	float target_gamespeed = 1.0f;
	bool  pause_sim = false;

	// [0,1] -> [0,24] hours
	// citizen schedules should be based on this, so changing it shouldn't mess up things if possible
	// (have them leave work if this is set?)
	float time_of_day = 0.6f;
	float day_duration = 10.0f * 60.0f; // day length in seconds of game time
	bool  pause_day = false;

	// [0,1] -> January->December
	float time_of_year = 0.4f;
	float year_duration = 60.0f * 60.0f; // year length in seconds of game time
	bool  pause_year = false;

	float planet_axial_tilt = deg(23.44f);
	float map_latitude = deg(35);
	float map_rotation = deg(30); // North default at +Y

	void progress_day (float eff_sim_dt) {
		if (pause_day) return;
		time_of_day += max(eff_sim_dt / day_duration, 0.0f);
		time_of_day = fmodf(time_of_day, 1.0f);
	}
	void progress_year (float eff_sim_dt) {
		if (pause_year) return;
		time_of_year += max(eff_sim_dt / year_duration, 0.0f);
		time_of_year = fmodf(time_of_year, 1.0f);
	}

	void update (Input& input) {
		if (input.buttons[KEY_SPACE].went_down)
			pause_sim = !pause_sim;

		float eff_speed = pause_sim ? 0.0f : target_gamespeed;
		float eff_dt = eff_speed * input.real_dt;
		progress_day(eff_dt);
		progress_year(eff_dt);
	}

	std::string formatted_date () {
		float t = time_of_year >= 1.0f ? 0.0f : time_of_year;
		int day = floori(time_of_year * (float)DAYS_IN_YEAR);

		int month = 0;
		int month_day = 0;
		for (int i=0; i<ARRLEN(MONTHS); ++i) {
			auto& m = MONTHS[i];
			if (day < m.first_day + m.days) {
				month = i;
				month_day = day - m.first_day;
				break;
			}
		}
		assert(month >= 0 && month < ARRLEN(MONTHS));
		assert(month_day >= 0 && month_day < MONTHS[month].days);

		return prints("%02d. %s", month_day+1, MONTHS[month].name);
	}
	std::string formatted_time () {
		float h = time_of_day >= 1.0f ? 0.0f : time_of_day * 24.0f;
		float hour   = floor(h);
		float minute = (h - hour) * 60.0f;

		assert((int)hour >= 0 && (int)hour < 24);
		assert((int)minute >= 0 && (int)minute < 60);

		return prints("%02d:%02d", (int)hour, (int)minute);
	}

	void imgui () {
		if (!imgui_Header("GameTime", true)) return;

		ImGui::Indent(100);
		ImGui::TextColored(ImVec4(0.95f, 0.95f, 0.8f, 1), "%s | %s", formatted_time().c_str(), formatted_date().c_str());
		ImGui::Unindent(100);
		
		ImGui::Separator();
			
		ImGui::Text("Simulation Speed");
		ImGui::SliderFloat("Target##target_gamespeed", &target_gamespeed, 0,100, "%.1fx", ImGuiSliderFlags_Logarithmic);
			
		ImGui::Checkbox("Pause [Space]", &pause_sim);
		ImGui::SameLine(0, 20); if (ImGui::Button("1"))    target_gamespeed = 1;
		ImGui::SameLine();      if (ImGui::Button(">"))    target_gamespeed = 2;
		ImGui::SameLine();      if (ImGui::Button(">>"))   target_gamespeed = 4;
		ImGui::SameLine();      if (ImGui::Button(">>>"))  target_gamespeed = 10;
		ImGui::SameLine();      if (ImGui::Button(">>>>")) target_gamespeed = 50;
		
		ImGui::Separator();

		if (ImGui::TreeNodeEx("Time of Day", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::SetNextItemWidth(-70.0f);
			ImGui::SliderFloat("##Time of Day", &time_of_day, 0,1);
			ImGui::SameLine();
			ImGui::Checkbox("Pause", &pause_day);
			ImGui::SetNextItemWidth(-120.0f);
			ImGui::SliderFloat("Duration (ingame)", &day_duration, 1, 60.0f*60*24, "%.0fs", ImGuiSliderFlags_Logarithmic);
			ImGui::TreePop();
		}
		ImGui::Separator();

		if (ImGui::TreeNodeEx("Time of Year", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::SetNextItemWidth(-70.0f);
			ImGui::SliderFloat("##Time of Year", &time_of_year, 0,1);
			ImGui::SameLine();
			ImGui::Checkbox("Pause", &pause_year);
			ImGui::SetNextItemWidth(-120.0f);
			ImGui::SliderFloat("Duration (ingame)", &year_duration, 1, 60.0f*60*24*365, "%.0fs", ImGuiSliderFlags_Logarithmic);
			ImGui::TreePop();
		}
		ImGui::Separator();
		
		if (ImGui::TreeNode("Details")) {
			ImGui::SliderAngle("Planet Axial Tilt", &planet_axial_tilt, -90, +90, "%.3f deg");
			ImGui::SliderAngle("Map Latitude", &map_latitude, -90, 90, "%.3f deg");
			ImGui::SliderAngle("Map Rotation", &map_rotation, 0, 360, "%.3f deg");
			ImGui::TreePop();
		}

		ImGui::PopID();
	}
	
	struct SkyConfig {
		// solar = solar system (not sun), useful for night sky cubemaps
		float3x3 sun2world, world2sun;
		float3x3 moon2world, world2moon;
		float3x3 solar2world, world2solar;
	};
	SkyConfig calc_sky_config (View3D& view) {
		SkyConfig res;

		// planet orbit adds one extra day per year
		float astro_time_of_year = time_of_year + DECEMBER_SOLSTICE;
		float sidereal_spin = time_of_day + astro_time_of_year;

		float moon_orbit_t = MOON_ORBITS_PER_YEAR * astro_time_of_year;
		
		float3x3 planet_tilt = rotate3_X(planet_axial_tilt);
		float3x3 planet_spin = rotate3_Z(sidereal_spin * deg(360));
		float3x3 map2planet = rotate3_X(deg(90) - map_latitude) * rotate3_Z(map_rotation);

		float3x3 planet_orbit     = rotate3_Z(astro_time_of_year * deg(360));
		float3x3 inv_planet_orbit = rotate3_Z(astro_time_of_year * -deg(360));

		float3x3 moon2solar = rotate3_X(MOON_ORBIT_INCLINATION) * rotate3_Z(moon_orbit_t * deg(360));
		float3x3 solar2moon = rotate3_Z(moon_orbit_t * -deg(360)) * rotate3_X(-MOON_ORBIT_INCLINATION);
		
		// Shadowmapping wants this matricies with Z pointing toward camera, and this is more intuitive for me as well
		// Z points from map towards sun/moon
		// Y points to sun/moon north pole
		// X points to left when looking towards sun/moon (flipped to make sure right handed coordinate system or else shadowmaps will draw backfaces!)
		float3x3 Z_TO_Y_OUTWARD = float3x3::rows(-1,0,0, 0,0,1, 0,1,0);
		//float3x3 Y_TO_Z_OUTWARD = inverse(Z_TO_Y_OUTWARD);
		float3x3& Y_TO_Z_OUTWARD = Z_TO_Y_OUTWARD; // don't bother inverting, same value

		res.world2solar = planet_tilt * planet_spin * map2planet;
		res.solar2world = inverse(res.world2solar);
		
		res.sun2world  = (res.solar2world * planet_orbit) * Z_TO_Y_OUTWARD;
		res.world2sun  = Y_TO_Z_OUTWARD * (inv_planet_orbit * res.world2solar);
		
		res.moon2world = (res.solar2world * moon2solar) * Z_TO_Y_OUTWARD;
		res.world2moon = Y_TO_Z_OUTWARD * (solar2moon * res.world2solar);
		
		//float3 sun_x   = res.sun2world * float3(1,0,0);
		//float3 sun_y   = res.sun2world * float3(0,1,0);
		//float3 sun_dir = res.sun2world * float3(0,0,1);
		//
		//float3 moon_x   = res.moon2world * float3(1,0,0);
		//float3 moon_y   = res.moon2world * float3(0,1,0);
		//float3 moon_dir = res.moon2world * float3(0,0,1);
		//
		//float3 solar_x = res.solar2world * float3(1,0,0);
		//float3 solar_y = res.solar2world * float3(0,1,0);
		//float3 solar_z = res.solar2world * float3(0,0,1);
		//
		//auto solar_col = lrgba(.4f,.5f,1, 1);
		//auto sun_col    = lrgba(.9f, 1, 0.2f, 1);
		//auto moon_col   = lrgba(.7f, .7f, 0.8f, 1);
		//
		//g_dbgdraw.arrow(view, 0, sun_dir * 5.0f, 0.1f, sun_col);
		//g_dbgdraw.arrow(view, sun_dir * 5.0f, sun_x, 0.1f, sun_col * lrgba(1,0.7f,0.7f, 1));
		//g_dbgdraw.arrow(view, sun_dir * 5.0f, sun_y, 0.1f, sun_col * lrgba(0.7f,1,0.7f, 1));
		//
		//g_dbgdraw.arrow(view, 0, moon_dir * 5.0f, 0.1f, moon_col);
		//g_dbgdraw.arrow(view, moon_dir * 5.0f, moon_x, 0.1f, moon_col * lrgba(1,0.7f,0.7f, 1));
		//g_dbgdraw.arrow(view, moon_dir * 5.0f, moon_y, 0.1f, moon_col * lrgba(0.7f,1,0.7f, 1));
		//
		//g_dbgdraw.arrow(view, 0, solar_z * 2.0f, 0.2f, solar_col);
		//g_dbgdraw.arrow(view, 0, solar_x * 2.0f, 0.2f, solar_col * lrgba(1,0.7f,0.7f, 1));
		//g_dbgdraw.arrow(view, 0, solar_y * 2.0f, 0.2f, solar_col * lrgba(0.7f,1,0.7f, 1));

		return res;
	}

	void visualize_planet (View3D& view) {
		// planet orbit adds one extra day per year
		float astro_time_of_year = time_of_year + DECEMBER_SOLSTICE;
		float sidereal_spin = time_of_day + astro_time_of_year;

		float moon_orbit_t = MOON_ORBITS_PER_YEAR * astro_time_of_year + 0.5f; // for some reason moon is opposite direction in the actual code, just fix it like this?
		
		// Map is at +Y of planet, rotated up or down based on latitude
		float3x3 map2planet = rotate3_X(deg(90) - map_latitude) * rotate3_Z(map_rotation);

		float3x3 planet_tilt = rotate3_X(planet_axial_tilt);
		float3x3 planet_spin = rotate3_Z(sidereal_spin * deg(360));
		float3 planet_orbit_pos = rotate3_Z(astro_time_of_year * deg(360)) * float3(0,-15,0); // translate planet by circular path, not translate then rotate, because that rotates planet along with orbit, causing axis tilt to rotate as well
		
		float3x4 planet2solar = translate(planet_orbit_pos) * planet_tilt * planet_spin;
		float3x4 map2solar = planet2solar * map2planet;

		// translate first, then orbit (then translate to planet), so that visualization is correctly tidally locked to planet
		float3x3 moon_orbit = rotate3_X(MOON_ORBIT_INCLINATION) * rotate3_Z(moon_orbit_t * deg(360));
		float3x4 moon2solar = translate(planet_orbit_pos) * moon_orbit * translate(float3(0,-5,0));

		float3x4 sun2solar = float3x4::identity();
		
		float3 dir2sun = normalizesafe(-planet_orbit_pos);
		float3 dir2moon = normalizesafe((moon2solar * float3(0)) - planet_orbit_pos);
		float3 map_pos = planet2solar * (map2planet * float3(0,0,1));
		
		static int center = 0;
		ImGui::Combo("Center", &center, "Sun\0Planet\0Map");
		if (center == 1) {
			float3x4 solar2planet = translate(-planet_orbit_pos);

			planet2solar = solar2planet * planet2solar;
			map2solar    = solar2planet * map2solar;
			moon2solar   = solar2planet * moon2solar;
			sun2solar    = solar2planet * sun2solar;
			
			dir2sun = (float3x3)solar2planet * dir2sun;
			dir2moon = (float3x3)solar2planet * dir2moon;
			map_pos = solar2planet * map_pos;
		} else if (center == 2) {
			float3x4 solar2map = (float3x4)inverse((float4x4)map2solar);

			planet2solar = solar2map * planet2solar;
			map2solar    = solar2map * map2solar;
			moon2solar   = solar2map * moon2solar;
			sun2solar    = solar2map * sun2solar;

			dir2sun = (float3x3)solar2map * dir2sun;
			dir2moon = (float3x3)solar2map * dir2moon;
			map_pos = solar2map * map_pos;
		}

		auto planet_col = lrgba(.4f,.5f,1, 1);
		auto sun_col    = lrgba(.9f, 1, 0.2f, 1);
		auto moon_col   = lrgba(.7f, .7f, 0.8f, 1);
		g_dbgdraw.wire_sphere(planet2solar, 1,     planet_col, 64, 32);
		g_dbgdraw.wire_sphere(sun2solar,    1,     sun_col,    64, 32);
		g_dbgdraw.wire_sphere(moon2solar,   0.33f, moon_col,   32, 16);

		g_dbgdraw.arrow(view, map_pos, ((float3x3)map2solar * float3(1,0,0)) * 0.3f, 0.03f, lrgba(1,0,0,1));
		g_dbgdraw.arrow(view, map_pos, ((float3x3)map2solar * float3(0,1,0)) * 0.3f, 0.03f, lrgba(0,1,0,1));
		g_dbgdraw.arrow(view, map_pos, ((float3x3)map2solar * float3(0,0,1)) * 0.3f, 0.03f, lrgba(0,0,1,1));

		g_dbgdraw.arrow(view, map_pos + dir2sun, -dir2sun * 0.4f, 0.03f, sun_col);
		g_dbgdraw.arrow(view, map_pos + dir2moon, -dir2moon * 0.4f, 0.03f, moon_col);
	}
};
