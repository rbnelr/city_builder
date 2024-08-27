#pragma once
#include "common.hpp"
#include "app.hpp"
#include "opengl.hpp"
#include "bindless_textures.hpp"

namespace ogl {

struct TexLoader {
	
	static std::string add_tex_suffix (std::string_view filename, std::string_view suffix) {
		std::string_view base;
		auto ext = kiss::get_ext(filename, &base, true);
		return kiss::concat(base, suffix, ext);
	}
	static std::string get_basename (std::string_view filepath) {
		// make sure to split first to avoid any '.' in path
		std::string_view filename;
		std::string_view path = kiss::get_path(filepath, &filename);

		std::string_view base;
		kiss::split(filename, '.', &base, 1);
		
		return kiss::concat(path, base);
	}

	struct BaseJob {
		virtual ~BaseJob() {}
		// run on threads
		virtual void execute () = 0;
		// finish on main thread
		virtual void finish (BindlessTextureManager& bindless) = 0;
	};

	template <typename T>
	struct LoadTexture2D : public BaseJob {
		std::string name;
		bool mips;
		int bindless_slot;
		// results
		bool success = false;

		Image<T> img;

		LoadTexture2D (std::string&& name, bool mips, int bindless_slot): name{name}, mips{mips}, bindless_slot{bindless_slot} {}

		virtual void execute () {
			ZoneScoped;
			log("loading texture \"%s\"...\n", name.c_str());

			std::string filepath = prints("assets/%s", name.c_str());

			if (!Image<T>::load_from_file(filepath.c_str(), &img)) {
				log_warn("Error! Could not load texture \"%s\"\n", name.c_str());
				return;
			}

			success = true;
		}

		virtual void finish (BindlessTextureManager& bindless) {
			auto base_name = get_basename(name);
			if (success) {
				// HACK: or is it? Remove .png etc from filename before plugging into bindless textures, 
				bindless.upload_texture<T>(base_name, bindless_slot, img, mips);
			}
		}
	};
	template <typename T>
	struct LoadCubemap : public BaseJob {
		std::string name_fmt;
		bool mips;
		TextureCubemap* result_ptr; // need resulting texture due to not having bindless cubemaps
		// results
		bool success = false;

		bool is_dds;

		Image<T> imgs[6];
		dds::Image ddss[6];
		
		LoadCubemap (std::string&& name_fmt, bool mips, TextureCubemap* result_ptr): name_fmt{name_fmt}, mips{mips}, result_ptr{result_ptr} {}

		virtual void execute () {
			ZoneScoped;
			log("loading cubemap \"%s\"...\n", name_fmt.c_str());
			
			std::string filepath_fmt = prints("assets/%s", name_fmt.c_str());
			
			is_dds = kiss::get_ext(filepath_fmt) == "dds";
			if (is_dds) {
				for (int i=0; i<6; ++i) {
					auto filepath = prints(filepath_fmt.c_str(), CUBEMAP_FACE_FILES_NAMES[i]);
			
					if (dds::readFile(filepath, &ddss[i]) != dds::ReadResult::Success) {
						log_warn("Error! Could not load texture \"%s\"\n", filepath.c_str());
						return;
					}
				}
			}
			else {
				for (int i=0; i<6; ++i) {
					auto filepath = prints(filepath_fmt.c_str(), CUBEMAP_FACE_FILES_NAMES[i]);
			
					if (!Image<T>::load_from_file(filepath.c_str(), &imgs[i])) {
						log_warn("Error! Could not load texture \"%s\"\n", filepath.c_str());
						return;
					}
				}
			}

			success = true;
		}

		virtual void finish (BindlessTextureManager& bindless) {
			if (!success) return;

			if (is_dds) {
				*result_ptr = {std::string_view(name_fmt)};
				ogl::upload_imageCube_DDS(*result_ptr, ddss, mips);
			}
			else {
				*result_ptr = {std::string_view(name_fmt)};
				ogl::upload_imageCube(*result_ptr, imgs, mips);
			}
		}
	};

	const int num_lod_threads = max(std::thread::hardware_concurrency()-2, 2);
	Threadpool<BaseJob> tex_load_threadpool = Threadpool<BaseJob>(num_lod_threads, TPRIO_BACKGROUND, "texload threads");

	void kickoff_loads (std::vector<std::unique_ptr<BaseJob>>&& loads) {
		tex_load_threadpool.jobs.push_n(loads.data(), loads.size());
	}
	void wait_for_finish_loads (BindlessTextureManager& bindless, int num_results) {
		//tex_load_threadpool.contribute_work(); // Don't contribute work because we should rather just spend the time uploading as soon as possible
		for (int i=0; i<num_results; ++i) {
			auto res = tex_load_threadpool.results.pop_wait();
			res->finish(bindless);
			// Job object gets destroyed
		}
	}
};

struct HeightmapTextures {

	struct Zone {
		std::string_view label;

		Texture2D tex;
		int2 size = -1;

		GLuint pbo = 0;

		void recreate (int2 size) {
			tex = {label};
			this->size = size;

			glBindTexture(GL_TEXTURE_2D, tex);
			glTexStorage2D(GL_TEXTURE_2D, 1, GL_R16, size.x, size.y);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
			glBindTexture(GL_TEXTURE_2D, 0);

			glGenBuffers(1, &pbo);
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);
			OGL_DBG_LABEL(GL_BUFFER, pbo, kiss::concat(label, ".pbo"));
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
		}
		~Zone () {
			if (pbo) glDeleteBuffers(1, &pbo);
		}
		
		void update (Heightmap::HeightmapZone& heightmap) {
			if (size != heightmap.data.size)
				recreate(heightmap.data.size);

			if (!heightmap.dirty_rect.empty())
				upload_rect(heightmap);
		}
		void upload_rect (Heightmap::HeightmapZone& heightmap) {
			auto& rect = heightmap.dirty_rect;
			assert(rect.lo.x <= rect.hi.x && rect.lo.y <= rect.hi.y);
			assert(rect.lo.x >= 0 && rect.lo.y >= 0 && rect.hi.x < size.x && rect.hi.y < size.y);

			int2 rect_size = rect.hi+1 - rect.lo;
			
			size_t mem_size = (size_t)rect_size.x * rect_size.y * sizeof(Heightmap::pixel_t);
			
			glBindTexture(GL_TEXTURE_2D, tex);
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);
			glBufferData(GL_PIXEL_UNPACK_BUFFER, mem_size, nullptr, GL_STREAM_DRAW);

			auto* mapped = (Heightmap::pixel_t*)glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY);
			assert(mapped);
			
			for (int y=0; y<rect_size.y; ++y) {
				memcpy(mapped + y*rect_size.x,
					heightmap.data.pixels + rect.lo.x + (rect.lo.y+y)*heightmap.data.size.x,
					sizeof(Heightmap::pixel_t)*rect_size.x);
			}

			glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);

			glTexSubImage2D(GL_TEXTURE_2D, 0, rect.lo.x, rect.lo.y, rect_size.x, rect_size.y, GL_RED, GL_UNSIGNED_SHORT, nullptr);

			glBindTexture(GL_TEXTURE_2D, 0);
			glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
			glInvalidateBufferData(GL_PIXEL_UNPACK_BUFFER);

			heightmap.reset_invalidate();
		}
	};
	Zone inner = {"heightmap.inner"};
	Zone outer = {"heightmap.outer"};

	void update_changes (Heightmap& heightmap) {
		//OGL_TRACE("test");

		inner.update(heightmap.inner);
		outer.update(heightmap.outer);
	}
		
	TextureBinds textures () {
		return {{
			{"heightmap_inner", inner.tex},
			{"heightmap_outer", outer.tex},
		}};
	}
};

struct Textures {
	TexLoader loader;
	
	// TODO: need some sort of texture array or atlas system! (atlas sucks tho)
	// -> find out if there is a modern system for single drawcall many textures (of differing sizes)
	// because it would be wierd that you can go all out with indirect instanced drawing, yet have to adjust your textures
	// just to be able to use them with one texture array

	BindlessTextureManager bindless_textures;

	Sampler sampler_normal  = make_sampler("sampler_normal", FILTER_MIPMAPPED, GL_REPEAT, true);
	Sampler sampler_cubemap = make_sampler("sampler_cubemap", FILTER_MIPMAPPED, GL_CLAMP_TO_EDGE);

	HeightmapTextures heightmap;

	typedef std::vector<std::unique_ptr<TexLoader::BaseJob>> Jobs;


	template <typename T>
	void texture (Jobs& jobs, std::string&& name, bool mips=true) {
		bindless_textures.add_entry(TexLoader::get_basename(name));
		jobs.push_back(std::make_unique<TexLoader::LoadTexture2D<T>>(std::move(name), mips, 0));
	}
	template <typename T>
	void texture_norm (Jobs& jobs, std::string&& name, bool mips=true, float uv_scale=1) {
		int id = bindless_textures.add_entry(TexLoader::get_basename(name));
		bindless_textures.entries[id].uv_scale = uv_scale; // TODO: maybe do this differently?

		auto norm = TexLoader::add_tex_suffix(name, ".norm");
		jobs.push_back(std::make_unique<TexLoader::LoadTexture2D<T>>(std::move(name), mips, 0));
		jobs.push_back(std::make_unique<TexLoader::LoadTexture2D<T>>(std::move(norm), mips, 1));
	}
	template <typename T>
	void texture_pbr (Jobs& jobs, std::string&& name, bool mips=true) {
		bindless_textures.add_entry(TexLoader::get_basename(name));

		auto pbr = TexLoader::add_tex_suffix(name, ".pbr");
		jobs.push_back(std::make_unique<TexLoader::LoadTexture2D<T>>(std::move(name), mips, 0));
		jobs.push_back(std::make_unique<TexLoader::LoadTexture2D<T>>(std::move(pbr ), mips, 2));
	}
	template <typename T>
	void texture_pbr_glow (Jobs& jobs, std::string&& name, bool mips=true) {
		bindless_textures.add_entry(TexLoader::get_basename(name));

		auto pbr  = TexLoader::add_tex_suffix(name, ".pbr");
		auto glow = TexLoader::add_tex_suffix(name, ".glow");
		jobs.push_back(std::make_unique<TexLoader::LoadTexture2D<T>>(std::move(name), mips, 0));
		jobs.push_back(std::make_unique<TexLoader::LoadTexture2D<T>>(std::move(pbr ), mips, 2));
		jobs.push_back(std::make_unique<TexLoader::LoadTexture2D<T>>(std::move(glow), mips, 3));
	}

	template <typename T>
	void cubemap (Jobs& jobs, std::string&& name, bool mips, TextureCubemap* result_ptr) {
		jobs.push_back(std::make_unique<TexLoader::LoadCubemap<T>>(std::move(name), mips, result_ptr));
	}

	// These are not treated as non-bindless for now
	TextureCubemap night_sky;

	Texture2D* clouds = nullptr;
	Texture2D* moon = nullptr;
	Texture2D* moon_nrm = nullptr;

	Texture2D* grid = nullptr;
	Texture2D* terrain_diffuse = nullptr;

	Texture2D* cracks = nullptr;
	
	// TODO: fix this, automatically add .norm and also just encode these in the assets?
	std::string_view asphalt        = "ground/pebbled_asphalt";
	std::string_view pavement       = "ground/stone_flooring";
	std::string_view curb           = "ground/curb2";
	
	const char* turn_arrows[7] = {
		"misc/turn_arrow_L"  ,
		"misc/turn_arrow_S"  ,
		"misc/turn_arrow_LS" ,
		"misc/turn_arrow_R"  ,
		"misc/turn_arrow_LR" ,
		"misc/turn_arrow_SR" ,
		"misc/turn_arrow_LSR",
	};

	void reload_all () {
		ZoneScoped;
		
		bindless_textures.clear();

		// TODO: encode this in assets json somehow, part of this is already specified in assets as tex_filename
		// the others could be a generic list of misc texture
		// then need to consider if we simply want to automatically detect .norm .pbr etc in asset folder and load them?
		// what if shader needs more or less than those specific textures? actually specify this in file?
		// maybe like "skybox/moon.png & norm" to keep it a single string?
		// Or just define standard texture suffixes and just keep slots empty inside bindless lut for the (few) non-pbr materials?
		Jobs j;
		texture     <srgba8>(j, "skybox/clouds.png");
		texture_norm<srgb8 >(j, "skybox/moon.png");
		cubemap     <srgba8>(j, "skybox/night_sky/%s.dds", true, &night_sky);
		texture<srgba8>(j, "misc/grid2.png");
		texture<srgb8 >(j, "ground/Rock_Moss.jpg");
		texture<srgb8 >(j, "misc/cracks.png");

		texture<srgba8>(j, "misc/line.png"       );
		texture<srgba8>(j, "misc/stripe.png"     );
		texture<srgba8>(j, "misc/shark_teeth.png");
		for (auto& fn : turn_arrows)
			texture<srgba8>(j, concat(fn, ".png"));
		
		texture_norm<srgb8 >(j, concat(asphalt, ".png"));
		texture_norm<srgb8 >(j, concat(pavement, ".png"), true, 1.5f);
		texture_norm<srgb8 >(j, concat(curb, ".png"));
		
		texture_pbr_glow<srgb8 >(j, "vehicles/car.png");
		texture_pbr_glow<srgb8 >(j, "vehicles/bus.png");
		texture_pbr<srgb8 >(j, "buildings/house.png");
		texture_pbr<srgb8 >(j, "props/traffic_light.png");

		int num_results = (int)j.size();
		loader.kickoff_loads(std::move(j));
		loader.wait_for_finish_loads(bindless_textures, num_results);

		clouds    = bindless_textures.get_gl_tex("skybox/clouds", 0);
		moon      = bindless_textures.get_gl_tex("skybox/moon", 0);
		moon_nrm  = bindless_textures.get_gl_tex("skybox/moon", 1);
		grid      = bindless_textures.get_gl_tex("misc/grid2", 0);
		terrain_diffuse = bindless_textures.get_gl_tex("ground/Rock_Moss", 0);
		cracks    = bindless_textures.get_gl_tex("misc/cracks", 0);
	}

	Textures () {
		reload_all();
	}

	void imgui () {
		if (ImGui::Button("Reload All Textures"))
			reload_all();
	}
};

} // namespace ogl
