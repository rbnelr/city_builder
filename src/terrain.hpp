#pragma once
#include "common.hpp"

// This heightmap implementation currently has the problem that 16 bit precision is not enough to to
// small scale terraforming, because the delta heights round to 0!
// No idea what the solution is, could switch to float values everywhere, but heightmap already takes lots of memory
// Hybrid float and 16bit are probably too complicated and bug prone (remember can't ever move terrain just by save/loading etc., or roadas might get buried)

class Heightmap {
	friend SERIALIZE_TO_JSON(Heightmap)   { SERIALIZE_TO_JSON_EXPAND(inner, outer, height_min, height_range)
		t.save_binary();
	}
	friend SERIALIZE_FROM_JSON(Heightmap) {
		t = {}; // reset heightmap, if load_heightmap_data fails, don't want prev map to stick around
		SERIALIZE_FROM_JSON_EXPAND(inner, outer, height_min, height_range)
		if (!t.load_binary())
			t.create_empty_map();
	}
	
public:
	typedef uint16_t pixel_t;

	struct HeightmapZone {
		SERIALIZE(HeightmapZone, map_size);

		int2 map_size = -1;
		Image<pixel_t> data;

		// cleared by renderer
		// NOTE: hi is inclusive
		RectInt dirty_rect = RectInt::INF;
		
		void mem_use (MemUse& mem) {
			size_t data_sz = sizeof(pixel_t) * data.size.x * data.size.y;
			mem.add("HeightmapZone", sizeof(*this) + data_sz);
		}

		void set_empty (int2 resolution=1024, pixel_t init_value=UINT16_MAX/10) {
			data = Image<pixel_t>(resolution);
			data.clear(init_value);
			invalidate();
		}

		void invalidate (RectInt rect=RectInt::INF) {
			dirty_rect.add(rect);
			dirty_rect.lo = max(dirty_rect.lo, int2(0));
			dirty_rect.hi = min(dirty_rect.hi, data.size-1);
		}
		void reset_invalidate () {
			dirty_rect = RectInt::EMPTY;
		}

		float sample_bilinear_clamped (float2 const& uv) const {
			assert(data.size.x >= 2 && data.size.y >= 2);

			float2 xy = (float2)data.size * uv - 0.5f;
			float2 texel = floor(xy);
			float2 t = xy - texel;

			int2 c0 = clamp((int2)texel,   int2(0), data.size-1);
			int2 c1 = clamp((int2)texel+1, int2(0), data.size-1);
			float a = data.pixels[c0.x + c0.y*data.size.x];
			float b = data.pixels[c1.x + c0.y*data.size.x];
			float c = data.pixels[c0.x + c1.y*data.size.x];
			float d = data.pixels[c1.x + c1.y*data.size.x];

			return lerp(lerp(a, b, t.x), lerp(c, d, t.x), t.y);
		}
		
		template <typename FUNC>
		_FORCEINLINE void edit_in_rect (Heightmap& map, float2 pos, float radius, FUNC func) {
			// do all math in texel coords
			float2 center = (pos / (float2)map_size + 0.5f) * (float2)data.size;
			float2 rad = (radius / (float2)map_size) * (float2)data.size; // 2d because map could potentially be non-square and have non square pixels; weird.

			int2 lo = clamp(floori(center - rad), int2(0), data.size);
			int2 hi = clamp( ceili(center + rad), int2(0), data.size); // exclusive

			for (int y=lo.y; y<hi.y; ++y)
			for (int x=lo.x; x<hi.x; ++x) {
				float2 texel_center = (float2)int2(x,y) + 0.5f;
				float2 uv = (texel_center - center) / rad; // uv in [-1,+1]
				
				float height = (float)data.get(x, y) / (float)UINT16_MAX * map.height_range + map.height_min;
				height = func(height, uv);
				data.get(x, y) = (pixel_t)roundi(clamp((height - map.height_min) / map.height_range, 0.0f, 1.0f) * (float)UINT16_MAX);
			}

			if (hi.x > lo.x && hi.y > lo.y)
				dirty_rect.add(RectInt{ lo, hi-1 });
		}
		
		template <typename FUNC>
		_FORCEINLINE void gaussian_blur_rect (Heightmap& map, float2 pos, float radius, float sigma, FUNC func) {
			// do all math in texel coords
			float2 center = (pos / (float2)map_size + 0.5f) * (float2)data.size;
			float2 rad = (radius / (float2)map_size) * (float2)data.size; // 2d because map could potentially be non-square and have non square pixels; weird.

			int2 lo = clamp(floori(center - rad), int2(0), data.size);
			int2 hi = clamp( ceili(center + rad), int2(0), data.size); // exclusive
			int2 size = hi - lo;

			// gaussian blur function (how much pixels affect other based on distance) technically never reaches zero
			// but at radius=3*sigma it reaches ~0.3% contribution, which should be negligble
			int kernel_size = ceili(sigma * 3.0f);

			auto gauss_weight = [] (float x, float sigma) {
				return exp(x*x / (-2.0f * sigma*sigma));
			};

			float* copy  = new float[size.x * size.y];
			float* passX = new float[size.x * size.y];
			float* passY = new float[size.x * size.y];
			
			for (int y=0; y<size.y; ++y)
			for (int x=0; x<size.x; ++x) {
				copy[y*size.x + x] = (float)data.get(x + lo.x, y + lo.y);
			}

			for (int y=0; y<size.y; ++y)
			for (int x=0; x<size.x; ++x) {
				int kernLo = max(x - kernel_size, 0);
				int kernHi = min(x + kernel_size + 1, size.x);

				float sum = 0, total_weight = 0;
				for (int i=kernLo; i<kernHi; ++i) {
					float weight = gauss_weight((float)abs(i - x), sigma);
					float val = (float)copy[y*size.x + i];
					sum += val * weight;
					total_weight += weight;
				}

				passX[y*size.x + x] = sum / total_weight;
			}
			
			for (int y=0; y<size.y; ++y)
			for (int x=0; x<size.x; ++x) {
				int kernLo = max(y - kernel_size, 0);
				int kernHi = min(y + kernel_size + 1, size.y);

				float sum = 0, total_weight = 0;
				for (int i=kernLo; i<kernHi; ++i) {
					float weight = gauss_weight((float)abs(i - y), sigma);
					float val = (float)passX[i*size.x + x];
					sum += val * weight;
					total_weight += weight;
				}

				passY[y*size.x + x] = sum / total_weight;
			}

			for (int y=lo.y; y<hi.y; ++y)
			for (int x=lo.x; x<hi.x; ++x) {
				float2 texel_center = (float2)int2(x,y) + 0.5f;
				float2 uv = (texel_center - center) / rad; // uv in [-1,+1]
				
				float val = (float)data.get(x,y);
				val = func(val, passY[(x-lo.x) + (y-lo.y)*size.x], uv);
				data.get(x,y) = (pixel_t)roundi(clamp(val, 0.0f, (float)UINT16_MAX));
			}

			delete[] copy;
			delete[] passX;
			delete[] passY;

			if (hi.x > lo.x && hi.y > lo.y)
				dirty_rect.add(RectInt{ lo, hi-1 });
		}
	};

	HeightmapZone inner = { 16*1024 };
	HeightmapZone outer = { 128*1024 };

	int2 map_size () { return inner.map_size; }

	float height_min = -150;
	float height_range = (float)UINT16_MAX * 0.05f;

	float sample_height (float2 pos) const {
		float val;
		if (  abs(pos.x / (float)inner.map_size.x) <= 0.5 &&
			  abs(pos.y / (float)inner.map_size.y) <= 0.5  ) {
			float2 uv = pos / (float2)inner.map_size + 0.5f;
			val = inner.sample_bilinear_clamped(uv);
		}
		else {
			float2 uv = pos / (float2)outer.map_size + 0.5f;
			val = outer.sample_bilinear_clamped(uv);
		}
		return val * (1.0f / (float)UINT16_MAX) * height_range + height_min;
	}

	// TODO: Actually have this return a resonable height based on cursor position
	// Raymarch and sample_height each time? Can we avoid huge number of steps?
	std::optional<float3> raycast_cursor (View3D& view, Input& input) {
		ZoneScoped;
	#if 1
		Ray ray;
		if (!view.cursor_ray(input, &ray.pos, &ray.dir))
			return {};

		float hit_t;
		if (!intersect_ray_zplane(ray, 0.0f, &hit_t))
			return {};

		float2 pos2d = (float2)(ray.pos + ray.dir * hit_t);
		float z = sample_height(pos2d);
		return float3(pos2d, z);
	#else
		// This is wayyy too slow!
		// And not even completely accurate
		// could accelarate using min/max heights in a larger grid
		// but edge case of shallow rays should still avoid doing sample_bilinear_clamped, since its super slow as well
		Ray ray;
		if (!view.cursor_ray(input, &ray.pos, &ray.dir))
			return {};

		float2 texel_size = (float2)inner.map_size / (float2)inner.data.size;
		float step = min(texel_size.x, texel_size.y);

		float t = 0;
		for (;;) {
			float3 pos = ray.pos + ray.dir * t;
			if (  abs(pos.x) > inner.map_size.x*0.5f ||
				  abs(pos.y) > inner.map_size.y*0.5f ||
				  pos.z < height_min || pos.z > height_min + height_range)
				break;

			float h = sample_height((float2)pos);
			if (pos.z < h) {
				// try to account for overshoot
				//float overshoot = h - pos.z;
				//float overshoot_t = overshoot / ray.dir.z;
				//t -= overshoot_t;
				break;
			}

			t += step;
		}

		float2 xy = (float2)(ray.pos + ray.dir * t);
		float z = sample_height(xy);
		return float3(xy, z);
	#endif
	}

	template <typename FUNC>
	_FORCEINLINE void edit_in_rect (float2 pos, float radius, FUNC func) {
		inner.edit_in_rect(*this, pos, radius, func);
	}
	template <typename FUNC>
	_FORCEINLINE void gaussian_blur_rect (float2 pos, float radius, float sigma, FUNC func) {
		inner.gaussian_blur_rect(*this, pos, radius, sigma, func);
	}
	
	void imgui () {
		if (!imgui_Header("Heightmap")) return;

		ImGui::DragInt("Inner Map Size", &inner.map_size.x, 1.0f);
		ImGui::DragInt("Outer Map Size", &outer.map_size.x, 1.0f);

		ImGui::DragFloat("Height Min", &height_min, 0.1f);
		ImGui::DragFloat("Height Range", &height_range, 0.1f);

		if (ImGui::Button("Import")) {
			ImGui::OpenPopup("import_popup");
		}
		if (ImGui::BeginPopup("import_popup")) {
			ImGui::SeparatorText("Select files to import for inner and outter heightmap");

			static std::string inner_filename, outer_filename;

			ImGui::Text("Inner Map:");
			ImGui::SetNextItemWidth(-70);
			ImGui::InputText("##inner_filename", &inner_filename);
			ImGui::SameLine();
			if (ImGui::Button("Select##select_inner_filename")) {
				kiss::file_open_dialog(&inner_filename);
			}

			ImGui::Text("Outer Map:");
			ImGui::SetNextItemWidth(-70);
			ImGui::InputText("##outer_filename", &outer_filename);
			ImGui::SameLine();
			if (ImGui::Button("Select##select_outer_filename")) {
				kiss::file_open_dialog(&outer_filename);
			}

			ImGui::Spacing();

			if (ImGui::Button("Import")) {
				if (import(inner_filename.c_str(), outer_filename.c_str())) {
					ImGui::CloseCurrentPopup();
				}
			}

			ImGui::EndPopup();
		}

		ImGui::PopID();
	}

	void mem_use (MemUse& mem) {
		inner.mem_use(mem);
		outer.mem_use(mem);
	}

////
	void create_empty_map () {
		inner.set_empty();
		outer.set_empty();
	}

	bool import (const char* inner_filename, const char* outer_filename) {
		ZoneScoped;
		log("loading heightmap...\n");

		Image<pixel_t> in, out;
		bool success = Image<pixel_t>::load_from_file("assets/heightmap.png", &in);
		success =      Image<pixel_t>::load_from_file("assets/heightmap_outer.png", &out) && success;

		if (!success || in.size.x < 2 || in.size.y < 2 || out.size.x < 2 || out.size.y < 2) {
			log_error("Error! Could not load heightmap!\n");
			return false;
		}

		inner.data = std::move(in);
		outer.data = std::move(out);
		inner.invalidate();
		outer.invalidate();
		return true;
	}
	// TODO: add export

	// TODO: store this in root folder (alongside debug.json) for now
	// but in near future will add real city building functions, which I hope to have saved/loaded from dedicated savegames (folders)
	// at that point load and save will be called by Savegame object which passes us a path?
	static constexpr const char* heightmap_binary_filename = "heightmap.data";
	
	struct HeightmapFile {
		char filetag[4] = {'H','X','H','M'};
		int version = 1;

		int2 inner_size;
		int2 outer_size;

		//uint16_t inner_data[inner_size.y][inner_size.x];
		//uint16_t outer_data[outer_size.y][outer_size.x];
	};
	bool save_binary () const {
		ZoneScoped;

		auto file = fopen(heightmap_binary_filename, "wb");
		if (!file) return false;
		defer( fclose(file); );

		HeightmapFile header;
		header.inner_size = inner.data.size;
		header.outer_size = outer.data.size;
		if (fwrite(&header, sizeof(header),1, file) != 1)
			return false;

		if (fwrite(inner.data.pixels, sizeof(pixel_t) * inner.data.size.x * inner.data.size.y, 1, file) != 1)
			return false;
		
		if (fwrite(outer.data.pixels, sizeof(pixel_t) * outer.data.size.x * outer.data.size.y, 1, file) != 1)
			return false;

		return true;
	}
	bool load_binary () {
		ZoneScoped;

		auto file = fopen(heightmap_binary_filename, "rb");
		if (!file) return false;
		defer( fclose(file); );

		HeightmapFile header;
		if (fread(&header, sizeof(header),1, file) != 1)
			return false;
		
		HeightmapFile test;
		if (memcmp(&header.filetag, &test.filetag, sizeof(header.filetag)) != 0 ||
				header.version != test.version)
			return false; // old file version!

		auto tmp_inner = Image<pixel_t>(header.inner_size);
		auto tmp_outer = Image<pixel_t>(header.outer_size);

		if (fread(tmp_inner.pixels, sizeof(pixel_t) * tmp_inner.size.x * tmp_inner.size.y, 1, file) != 1)
			return false;
		
		if (fread(tmp_outer.pixels, sizeof(pixel_t) * tmp_outer.size.x * tmp_outer.size.y, 1, file) != 1)
			return false;

		if (tmp_inner.size.x < 2 || tmp_inner.size.y < 2 || tmp_outer.size.x < 2 || tmp_outer.size.y < 2)
			return false;
		
		inner.data = std::move(tmp_inner);
		outer.data = std::move(tmp_outer);
		inner.invalidate();
		outer.invalidate();
		return true;
	}
};

////

// TODO: create a single key bind class
inline constexpr Button TerraformToolButton = MOUSE_BUTTON_LEFT;
inline constexpr Button TerraformToolButton2 = MOUSE_BUTTON_RIGHT;

// radius from [0,1] to brush falloff
struct TerraformBrush {
	//SERIALIZE(TerraformBrush, exponent)
	
	//float exponent = 3;
	inline float calc (float r) {
		//return 1.0f - powf(r, exponent);
		return 1.0f - smoothstep(r);
	}
};

class TerraformTool {
public:
	virtual const char* name () = 0;
	virtual void imgui (Heightmap& map) = 0;
	virtual void edit_terrain (Heightmap& map, float3 cursor_pos, float radius, TerraformBrush& brush, Input& input) = 0;
};

class TerraformMove : public TerraformTool {
	SERIALIZE(TerraformMove, strength)

	float strength = 1;
public:
	virtual const char* name () { return "Move"; };
	virtual void imgui (Heightmap& map) {
		ImGui::SliderFloat("Strength", &strength, 0,10, "%.1f", ImGuiSliderFlags_Logarithmic);
	}
	virtual void edit_terrain (Heightmap& map, float3 cursor_pos, float radius, TerraformBrush& brush, Input& input) {
		bool add = input.buttons[TerraformToolButton].is_down;
		bool sub = input.buttons[TerraformToolButton2].is_down;
		if (!add && !sub)
			return;
		ZoneScoped;

		float speed = (add ? +1.0f : -1.0f) * strength * radius * input.real_dt;

		map.edit_in_rect(cursor_pos, radius, [&] (float height, float2 uv) {
			float r = length(uv);
			if (r < 1.0f) {
				height += brush.calc(r) * speed;
			}
			return height;
		});
	}
};
class TerraformFlatten : public TerraformTool {
	SERIALIZE(TerraformFlatten, target_height, strength, restrict_direction)

	float target_height = 0;
	float strength = 10;
	int restrict_direction = 0;

	// TODO: Add height visualization (transparent plane that goes more transparent behind terrain?)
public:
	virtual const char* name () { return "Flatten"; };
	virtual void imgui (Heightmap& map) {
		ImGui::SliderFloat("Height", &target_height, map.height_min, map.height_min + map.height_range);

		ImGui::SliderFloat("Strength", &strength, 0,10, "%.1f", ImGuiSliderFlags_Logarithmic);

		bool checked = restrict_direction > 0;
		if (ImGui::Checkbox("Raise Only", &checked)) restrict_direction = checked ? +1 : 0;
		ImGui::SameLine();

		checked = restrict_direction < 0;
		if (ImGui::Checkbox("Lower Only", &checked)) restrict_direction = checked ? -1 : 0;
	}
	virtual void edit_terrain (Heightmap& map, float3 cursor_pos, float radius, TerraformBrush& brush, Input& input) {
		if (input.buttons[TerraformToolButton2].is_down) {
			target_height = map.sample_height(cursor_pos);
		}
		else if (input.buttons[TerraformToolButton].is_down) {
			ZoneScoped;
			float speed = strength * radius * input.real_dt;
			
			map.edit_in_rect(cursor_pos, radius, [&] (float height, float2 uv) {
				float r = length(uv);
				if (r < 1.0f) {
					float delta = target_height - height;
					int dir = delta > 0.0f ? +1 : -1;
					if (restrict_direction == 0 || restrict_direction == dir) {
						height += min(abs(delta), brush.calc(r) * speed) * (float)dir; // move by constant speed up, without overshooting
					}
				}
				return height;
			});
		}
	}
};
class TerraformSmooth : public TerraformTool {
	SERIALIZE(TerraformSmooth, strength)

	float strength = 1;

	// TODO: Add height visualization (transparent plane that goes more transparent behind terrain?)
public:
	virtual const char* name () { return "Smooth"; };
	virtual void imgui (Heightmap& map) {
		ImGui::SliderFloat("Strength", &strength, 0,1, "%.1f", ImGuiSliderFlags_Logarithmic);
	}
	virtual void edit_terrain (Heightmap& map, float3 cursor_pos, float radius, TerraformBrush& brush, Input& input) {
		if (input.buttons[TerraformToolButton].is_down) {
			ZoneScoped;
			// does this make sense? higher sigma essentially means a larger blur radius (blur radius, not radius that we apply the blur to!)
			// gaussian blurs applied multiple times are equivalent to adding the sigma and doing one blur, so it should probably be scaled by dt
			// we also want larger radius (editing zoomed out) to blur more
			float gaussian_sigma = strength * radius * input.real_dt;
		
			map.gaussian_blur_rect(cursor_pos, radius, gaussian_sigma, [&] (float old_height, float smoothed_height, float2 uv) {
				float r = length(uv);
				if (r < 1.0f) {
					return lerp(old_height, smoothed_height, brush.calc(r));
				}
				return old_height;
			});
		}
	}
};

class HeightmapTerraform {
	//SERIALIZE(HeightmapTerraform, paint_radius, move_tool, flatten_tool, smooth_tool)
	// It feels better when these things are not kept across restarts, resonable defaults feel better?
	// cur_tool is also UI state, ie which button is clicked, which no one expect to be restored ever

	float paint_radius = 25;

	float3 radius_drag_pos;
	bool radius_dragging = false;

	bool dragging_lock_buttons = false; // avoid accedentally editing when finishing radius drag with mouse button

	TerraformBrush brush;

	TerraformMove    move_tool;
	TerraformFlatten flatten_tool;
	TerraformSmooth  smooth_tool;

	TerraformTool* cur_tool = nullptr;

	void update_and_show_edit_circle (float3 cursor_pos, Input& input) {
		paint_radius = max(paint_radius, 0.0f);
		
		float new_radius = 0;
		if (radius_dragging) {
			new_radius = length((float2)cursor_pos - (float2)radius_drag_pos);
			new_radius = max(new_radius, 0.0f);
			
			bool finish_dragging_LRMB =  input.buttons[MOUSE_BUTTON_LEFT].went_down || input.buttons[MOUSE_BUTTON_RIGHT].went_down;
			bool finish_dragging = input.buttons[KEY_R].went_down || finish_dragging_LRMB;
			bool cancel_dragging = input.buttons[KEY_ESCAPE].went_down;

			if (finish_dragging)
				paint_radius = new_radius;
			if (finish_dragging || cancel_dragging) {
				radius_dragging = false;
				if (finish_dragging_LRMB)
					dragging_lock_buttons = true;
				
				input.buttons[KEY_ESCAPE].went_down = false; // eat any esc presses to avoid double use in interact system
			}
		}
		else {
			bool begin_dragging = input.buttons[KEY_R].went_down;
			if (begin_dragging) {
				radius_drag_pos = cursor_pos;
				radius_dragging = true;
			}
		}
		
		if (radius_dragging) {
			g_dbgdraw.wire_circle(radius_drag_pos, paint_radius, lrgba(1,1,0.2f,0.2f), 128);
			g_dbgdraw.wire_circle(radius_drag_pos,   new_radius, lrgba(1,1,0.2f,1), 128);
		}
		else {
			g_dbgdraw.wire_circle(cursor_pos, paint_radius, lrgba(1,1,0.2f,1), 128);
		}

		if (input.buttons[MOUSE_BUTTON_LEFT].went_up || input.buttons[MOUSE_BUTTON_RIGHT].went_up)
			dragging_lock_buttons = false;
	}

public:
	void imgui (Heightmap& map) {
		ImGui::SeparatorText("Terraform");

		auto tool_button = [&] (TerraformTool* tool) {
			bool active = tool == cur_tool;
			//ImGui::Setstyle
			auto str = prints(active ? "[%s]###%s":"%s###%s", tool->name(), tool->name());
			if (ImGui::ButtonEx(str.c_str(), ImVec2(80, 20), ImGuiButtonFlags_PressedOnClick))
				cur_tool = !active ? tool : nullptr;
		};
		tool_button(&move_tool);
		ImGui::SameLine();
		tool_button(&flatten_tool);
		ImGui::SameLine();
		tool_button(&smooth_tool);

		ImGui::Text("Paint Radius  [R] to Drag-Change");
		ImGui::SliderFloat("##paint radius", &paint_radius, 0, 64*1024, "%.0f", ImGuiSliderFlags_Logarithmic);
		
		if (cur_tool)
			cur_tool->imgui(map);
	}
	
	void update (View3D& view, Input& input, Heightmap& map) {
		auto cursor_pos = map.raycast_cursor(view, input);
		if (!cursor_pos.has_value())
			return;
		
		update_and_show_edit_circle(cursor_pos.value(), input);

		if (!dragging_lock_buttons && cur_tool) {
			cur_tool->edit_terrain(map, cursor_pos.value(), paint_radius, brush, input);
		}
	}
};
