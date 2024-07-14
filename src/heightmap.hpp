#pragma once
#include "common.hpp"

struct Heightmap {
	int2 map_size       = 16*1024;
	int2 outer_map_size = 128*1024;

	float z_min = -150;
	float z_range = (float)UINT16_MAX * 0.05f;

	Image<uint16_t> inner, outer;

	bool textures_changed = false; // cleared by renderer

	Heightmap () {
		load_from_file();
	}

	void load_from_file () {
		ZoneScoped;
		printf("loading heightmap...\n");

		bool success = Image<uint16_t>::load_from_file("assets/heightmap.png", &inner);
		success =      Image<uint16_t>::load_from_file("assets/heightmap_outer.png", &outer) && success;

		if (!success) {
			fprintf(stderr, "Error! Could not load heightmap!\n");
		}

		textures_changed = true;
	}
	
	void imgui () {
		if (imgui_Header("Heightmap", true)) {
			ImGui::DragFloat("z_min", &z_min);
			ImGui::DragFloat("z_range", &z_range);

			ImGui::PopID();
		}
	}
};
