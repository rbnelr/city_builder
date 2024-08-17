#pragma once
#include "common.hpp"
#include "opengl.hpp"
#include "agnostic_render.hpp"
#include "bindless_textures.hpp"

namespace ogl {

static constexpr int UBO_BINDING_COMMON = 0;
static constexpr int SSBO_BINDING_BINDLESS_TEX_LUT = 0;
static constexpr int SSBO_BINDING_DBGDRAW_INDIRECT = 1;
static constexpr int SSBO_BINDING_ENTITY_INSTANCES = 2;
static constexpr int SSBO_BINDING_ENTITY_MDI       = 3;
static constexpr int SSBO_BINDING_ENTITY_MESH_INFO = 4;
static constexpr int SSBO_BINDING_ENTITY_LOD_INFO  = 5;

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
			printf("loading texture \"%s\"...\n", name.c_str());

			std::string filepath = prints("assets/%s", name.c_str());

			if (!Image<T>::load_from_file(filepath.c_str(), &img)) {
				fprintf(stderr, "Error! Could not load texture \"%s\"\n", name.c_str());
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
		Image<T> imgs[6];
		
		LoadCubemap (std::string&& name_fmt, bool mips, TextureCubemap* result_ptr): name_fmt{name_fmt}, mips{mips}, result_ptr{result_ptr} {}

		virtual void execute () {
			ZoneScoped;
			printf("loading cubemap \"%s\"...\n", name_fmt.c_str());
			
			std::string filepath_fmt = prints("assets/%s", name_fmt.c_str());

			for (int i=0; i<6; ++i) {
				auto filepath = prints(filepath_fmt.c_str(), CUBEMAP_FACE_FILES_NAMES[i]);

				if (!Image<T>::load_from_file(filepath.c_str(), &imgs[i])) {
					fprintf(stderr, "Error! Could not load texture \"%s\"\n", filepath.c_str());
					return;
				}
			}

			success = true;
		}

		virtual void finish (BindlessTextureManager& bindless) {
			if (success) {
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

struct Textures {
	TexLoader loader;
	
	// TODO: need some sort of texture array or atlas system! (atlas sucks tho)
	// -> find out if there is a modern system for single drawcall many textures (of differing sizes)
	// because it would be wierd that you can go all out with indirect instanced drawing, yet have to adjust your textures
	// just to be able to use them with one texture array

	BindlessTextureManager bindless_textures;

	Sampler sampler_normal = make_sampler("sampler_normal", FILTER_MIPMAPPED, GL_REPEAT, true);
	Sampler sampler_cubemap = make_sampler("sampler_cubemap", FILTER_MIPMAPPED, GL_CLAMP_TO_EDGE);

	struct HeightmapTextures {
		Texture2D inner, outer;

		Sampler sampler = make_sampler("sampler_heightmap", FILTER_MIPMAPPED, GL_CLAMP_TO_EDGE);
	
		void upload (Heightmap const& heightmap) {
			inner = {"heightmap.inner"};
			outer = {"heightmap.outer"};
			
			upload_image2D<uint16_t>(inner, heightmap.inner, true);
			upload_image2D<uint16_t>(outer, heightmap.outer, true);
		}
		
		TextureBinds textures () {
			return {{
				{"heightmap_inner", inner, sampler},
				{"heightmap_outer", outer, sampler},
			}};
		}
	};

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
		jobs.push_back(std::make_unique<TexLoader::LoadTexture2D<T>>(std::move(pbr), mips, 2));
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
	Texture2D* contours = nullptr;
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
		cubemap     <srgba8>(j, "skybox/night_sky/%s.png", true, &night_sky);
		texture<srgba8>(j, "misc/grid2.png");
		texture<srgba8>(j, "misc/contours.png");
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
		
		texture_pbr<srgb8 >(j, "vehicles/car.png");
		texture_pbr<srgb8 >(j, "vehicles/bus.png");
		texture_pbr<srgb8 >(j, "buildings/house.png");
		texture_pbr<srgb8 >(j, "props/traffic_light.png");

		int num_results = (int)j.size();
		loader.kickoff_loads(std::move(j));
		loader.wait_for_finish_loads(bindless_textures, num_results);

		clouds    = bindless_textures.get_gl_tex("skybox/clouds", 0);
		moon      = bindless_textures.get_gl_tex("skybox/moon", 0);
		moon_nrm  = bindless_textures.get_gl_tex("skybox/moon", 1);
		grid      = bindless_textures.get_gl_tex("misc/grid2", 0);
		contours  = bindless_textures.get_gl_tex("misc/contours", 0);
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

struct Gbuffer {
	Fbo fbo;

	// WARNING: can't really use alpha channel in gbuf because decal

#if 1 // 4+3+6+6+2 = 21 bytes
	// only depth -> reconstruct position  float32 format for resonable infinite far plane (float16 only works with ~1m near plane)
	static constexpr GLenum depth_format = GL_DEPTH_COMPONENT32F; // GL_R32F causes GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT
	// rgb albedo (emmisive is simply very bright)
	// might be able to afford using GL_RGB8 since it's mostly just albedo from textures
	static constexpr GLenum col_format   = GL_SRGB8; // GL_RGB16F GL_RGB8
	static constexpr GLenum emiss_format = GL_RGB16F;
	// rgb normal
	// GL_RGB8 requires encoding normals, might be better to have camera space normals without z
	// seems like 8 bits might be to little for normals
	static constexpr GLenum norm_format  = GL_RGB16_SNORM; // GL_RGB16F
	// PBR roughness, metallic
	static constexpr GLenum pbr_format   = GL_RG8;
#else // 4+3+4+3+2 = 16 bytes
	// Doesn't really make a difference? Not even going from renderscale 2x to 1x does much
	// Bottleneck vertex count / instance data currently?
	static constexpr GLenum depth_format = GL_DEPTH_COMPONENT32F;//GL_DEPTH_COMPONENT32F; // GL_R32F causes GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT
	static constexpr GLenum col_format   = GL_SRGB8;
	static constexpr GLenum emiss_format = GL_R11F_G11F_B10F;
	static constexpr GLenum norm_format  = GL_RGB8; // difference visible, but not by too much, octahedral encoding would fix it
	static constexpr GLenum pbr_format   = GL_RG8;
#endif

	Render_Texture depth  = {};
	Render_Texture col    = {};
	Render_Texture emiss  = {};
	Render_Texture norm   = {};
	Render_Texture pbr    = {};

	Sampler sampler = make_sampler("gbuf_sampler", FILTER_NEAREST, GL_CLAMP_TO_EDGE);

	void resize (int2 size) {
		glActiveTexture(GL_TEXTURE0);
		
		depth  = Render_Texture("gbuf.depth", size, depth_format);
		col    = Render_Texture("gbuf.col",   size, col_format);
		emiss  = Render_Texture("gbuf.emiss", size, emiss_format);
		norm   = Render_Texture("gbuf.norm",  size, norm_format);
		pbr    = Render_Texture("gbuf.pbr",   size, pbr_format);
		
		fbo = Fbo("gbuf.fbo");
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,  GL_TEXTURE_2D, depth, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, col, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, emiss, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, norm, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_2D, pbr, 0);
		
		// needed so layout(location = x) out vec3 frag_x; works
		GLuint bufs[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3 };
		glDrawBuffers(ARRLEN(bufs), bufs);
		
		GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
		if (status != GL_FRAMEBUFFER_COMPLETE) {
			fprintf(stderr, "glCheckFramebufferStatus: %x\n", status);
		}
		
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}

	TextureBinds textures () {
		return TextureBinds{{
			{ "gbuf_depth", { GL_TEXTURE_2D, depth }, sampler },
			{ "gbuf_col",   { GL_TEXTURE_2D, col   }, sampler },
			{ "gbuf_emiss", { GL_TEXTURE_2D, emiss }, sampler },
			{ "gbuf_norm",  { GL_TEXTURE_2D, norm  }, sampler },
			{ "gbuf_pbr",   { GL_TEXTURE_2D, pbr   }, sampler },
		}};
	}
};

struct PBR_Render {
	// GL_R11F_G11F_B10F is not noticably different and like 40% faster due to being memory read bottlenecked
	// *actually my starry night sky + moon skybox looks wonky and flashes, probably due to the values being too small for the format
	// TODO: dynamically adjust env map with exposure?
	static constexpr GLenum env_map_format = GL_R11F_G11F_B10F; // GL_R11F_G11F_B10F // unfortunately rgb16f is not supported!!!?, need to waste 1/4 of memory or use a non-functional format??
	static constexpr const char* env_map_format_compute = "r11f_g11f_b10f"; // GL_R11F_G11F_B10F

	Shader* shad_integrate_brdf = g_shaders.compile("pbr_integrate_brdf");

	static constexpr int3 COMPUTE_CONVOLVE_WG = int3(8,8,1);
	Shader* shad_compute_gen_env  = g_shaders.compile("pbr_compute", {{"MODE","0"}, {"ENV_PIXEL_FORMAT",env_map_format_compute}}, { shader::COMPUTE_SHADER });
	Shader* shad_compute_gen_mips = g_shaders.compile("pbr_compute", {{"MODE","1"}, {"ENV_PIXEL_FORMAT",env_map_format_compute}}, { shader::COMPUTE_SHADER });
	Shader* shad_compute_copy     = g_shaders.compile("pbr_compute", {{"MODE","2"}, {"ENV_PIXEL_FORMAT",env_map_format_compute}}, { shader::COMPUTE_SHADER });
	Shader* shad_compute_convolve = g_shaders.compile("pbr_compute", {{"MODE","3"}, {"ENV_PIXEL_FORMAT",env_map_format_compute}}, { shader::COMPUTE_SHADER });

	Texture2D brdf_LUT;
	int brdf_LUT_res = 256; // Seems to be enough
	bool recreate_brdf = true;
	
	TextureCubemap base_env_map, pbr_env_convolved;
	int env_res = 384;
	int env_mips = 8; // env_res=384 -> 3x3
	std::vector<int> sampler_per_res = std::vector<int>({ // num_samples for each resolution
		512,512,512,512,512,512, // 1,2,4,8,16,32
		512, // 64
		256, // 128
		256, // 256
		64, // 512
		64, // 1024
	});
	// adjust roughness with  roughness_mip = pow(roughness, env_roughness_curve)
	// to squeeze more of the low roughness into the lower mips, so we waste less blurry high roughness on the high-res mips
	// Note: technically makes interpolation non-linear (wrong), but probably is still more correct than a too blurry mip
	float env_roughness_curve = 0.6f;
	bool recreate_env_map = true;
	bool redraw_env_map = true;

	void imgui () {
		if (ImGui::TreeNode("PBR")) {
			recreate_brdf = ImGui::SliderInt("brdf_LUT_res", &brdf_LUT_res, 1, 1024, "%d", ImGuiSliderFlags_Logarithmic) || recreate_brdf;
			
			recreate_env_map = ImGui::SliderInt("env_res", &env_res, 1, 2048, "%d", ImGuiSliderFlags_Logarithmic) || recreate_env_map;
			
			int mips = calc_mipmaps(env_res, env_res);
			recreate_env_map = ImGui::SliderInt("env_mips", &env_mips, 1, mips, "%d") || recreate_env_map;
			env_mips = min(env_mips, calc_mipmaps(env_res, env_res));
			
			int lowest_res = env_res >> (env_mips-1);
			ImGui::Text("Env map lowest mip: %dx%d pixels", lowest_res, lowest_res);

			imgui_edit_vector("sampler_per_res", sampler_per_res, [&] (int i, int& item) {
				ImGui::Text("%4dx%4d mipmap", 1<<i, 1<<i); ImGui::SameLine();
				return ImGui::DragInt("##sampler_per_res", &sampler_per_res[i], 0.1f, 1,4*1024, "%d", ImGuiSliderFlags_Logarithmic);
			}, true, false, false);

			ImGui::TreePop();
		}
	}

	void create_brdf_lut (StateManager& state) {
		ZoneScoped;
		OGL_TRACE("create_brdf_lut");
			
		// recreate texture
		brdf_LUT = {"PBR.brdf_LUT"};
		glBindTexture(GL_TEXTURE_2D, brdf_LUT);

		glTexStorage2D(GL_TEXTURE_2D, 1, GL_RG16, brdf_LUT_res, brdf_LUT_res);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		{
			auto fbo = make_and_bind_temp_fbo(GL_TEXTURE_2D, brdf_LUT, 0);
			// clear for good measure
			glClearColor(0,0,0,0);
			glClear(GL_COLOR_BUFFER_BIT);
				
			glUseProgram(shad_integrate_brdf->prog);
			shad_integrate_brdf->set_uniform("resolution", float2((float)brdf_LUT_res));
				
			state.bind_textures(shad_integrate_brdf, {});
			draw_fullscreen_triangle(state);
		}

		glBindTexture(GL_TEXTURE_2D, 0);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}
	
	TextureCubemap create_cubemap (int res, int mips, std::string_view lbl) {
		ZoneScoped;
		OGL_TRACE("create_cubemap");
			
		// recreate texture
		TextureCubemap tex = {lbl};
		glBindTexture(GL_TEXTURE_CUBE_MAP, tex);

		// allocates all faces in a single call
		glTexStorage2D(GL_TEXTURE_CUBE_MAP, mips, env_map_format, res,res);

		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_BASE_LEVEL, 0);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAX_LEVEL, mips-1);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
		
		glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
		return tex;
	}

	void create_env_map () {
		base_env_map = create_cubemap(env_res, calc_mipmaps(env_res, env_res), "PBR.env_map");
		pbr_env_convolved = create_cubemap(env_res, env_mips, "PBR.pbr_env_convolved");
	}
	void draw_env_map (StateManager& state, Textures& texs) {
		ZoneScoped;
		OGL_TRACE("draw_env_map");

		int res = env_res;
		{
			OGL_TRACE("gen_env");

			glUseProgram(shad_compute_gen_env->prog);
			shad_compute_gen_env->set_uniform("resolution", int2(res));

			state.bind_textures(shad_compute_gen_mips, {
				{ "clouds", *texs.clouds },
				{ "night_sky", texs.night_sky, texs.sampler_cubemap },
				{ "moon", *texs.moon },
				{ "moon_nrm", *texs.moon_nrm },
			});

			glBindImageTexture(0, base_env_map, 0, GL_FALSE, 0, GL_WRITE_ONLY, env_map_format);

			dispatch_compute(int3(res,res,6), COMPUTE_CONVOLVE_WG);

			glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT|GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		}
		{
			OGL_TRACE("gen mips");
			glUseProgram(shad_compute_gen_mips->prog);
			state.bind_textures(shad_compute_gen_mips, {
				{ "base_env_map", base_env_map },
			});

			for (int mip=1; mip<env_mips; ++mip) {
				res = max(res / 2, 1);

				shad_compute_gen_mips->set_uniform("resolution", int2(res));
				shad_compute_gen_mips->set_uniform("prev_mip", (float)(mip-1));

				glBindImageTexture(0, base_env_map, mip, GL_FALSE, 0, GL_WRITE_ONLY, env_map_format);

				dispatch_compute(int3(res,res,6), COMPUTE_CONVOLVE_WG);
				
				glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT|GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
			}
		}
		
		res = env_res;
		{
			OGL_TRACE("copy mip0");

			glUseProgram(shad_compute_copy->prog);
			shad_compute_copy->set_uniform("resolution", int2(res));

			glBindImageTexture(0, pbr_env_convolved, 0, GL_FALSE, 0, GL_WRITE_ONLY, env_map_format);
			glBindImageTexture(1, base_env_map,      0, GL_FALSE, 0, GL_READ_ONLY,  env_map_format);

			dispatch_compute(int3(res,res,6), COMPUTE_CONVOLVE_WG);
		}
		
		{
			OGL_TRACE("convolve");
			glUseProgram(shad_compute_convolve->prog);
			state.bind_textures(shad_compute_convolve, {
				{ "base_env_map", base_env_map },
			});

			for (int mip=1; mip<env_mips; ++mip) {
				res = max(res / 2, 1);
				
				float t = (float)mip / (float)(env_mips-1);
				float roughness = powf(t, 1.0f / env_roughness_curve);
				shad_compute_convolve->set_uniform("roughness", roughness);
				shad_compute_convolve->set_uniform("resolution", int2(res));

				int num_samples = sampler_per_res[floori(log2f((float)res))];
				shad_compute_convolve->set_uniform("num_samples", num_samples);

				glBindImageTexture(0, pbr_env_convolved, mip, GL_FALSE, 0, GL_WRITE_ONLY, env_map_format);

				constexpr int BATCH_SIZE = 32;
				dispatch_compute(int3(BATCH_SIZE,res*res,6), int3(BATCH_SIZE,4,1));
			}
		}

		// TODO: Is this correct?
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT|GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

		glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		glBindVertexArray(0);
	}

	void update (StateManager& state, Textures& texs) {
		if (recreate_brdf) {
			create_brdf_lut(state);
			recreate_brdf = false;
		}
		if (recreate_env_map) {
			create_env_map();
			recreate_env_map = false;
		}
		if (redraw_env_map) {
			draw_env_map(state, texs);
			// redraw_env_map = false;
		}
	}

	void prepare_uniforms_and_textures (Shader* shad, TextureBinds& tex) {
		shad->set_uniform("pbr_env_map_last_mip", (float)(env_mips-1));
		shad->set_uniform("pbr_env_roughness_curve", env_roughness_curve);
		
		// no samplers, just use texture params instead
		tex += { "pbr_brdf_LUT", brdf_LUT };
		tex += { "pbr_env_map", pbr_env_convolved };
	}

#if 0
	void visualize_hemisphere_sampling () {
		static int _test_samples = 1000;
		static float _test_size = 0.05f;
		static int _test_scheme = 0;
		static float _roughness = 0.2f;
		ImGui::SliderInt("_test_samples", &_test_samples, 0, 2000);
		ImGui::SliderFloat("_test_size", &_test_size, 0, 1);
		ImGui::SliderInt("_test_scheme", &_test_scheme, 0, 3);
		ImGui::SliderFloat("_roughness", &_roughness, 0, 1);

		auto radical_inverse_VdC = [&] (uint32_t bits) {
			bits = ( bits                << 16u) | ( bits                >> 16u);
			bits = ((bits & 0x00FF00FFu) <<  8u) | ((bits & 0xFF00FF00u) >>  8u);
			bits = ((bits & 0x0F0F0F0Fu) <<  4u) | ((bits & 0xF0F0F0F0u) >>  4u);
			bits = ((bits & 0x33333333u) <<  2u) | ((bits & 0xCCCCCCCCu) >>  2u);
			bits = ((bits & 0x55555555u) <<  1u) | ((bits & 0xAAAAAAAAu) >>  1u);
			return float(bits) * 2.3283064365386963e-10f; // 1.0 / 0x100000000
		};
		auto hammersley = [&] (int i, int N) {
			return float2(float(i)/float(N), radical_inverse_VdC(uint32_t(i)));
		};
		auto sample_distribution = [&] (int i, int count) {
			float2 u = hammersley(i, count);

			float phi   = 2.0f * PI * u.x;

			float theta = 0;
			if      (_test_scheme == 0) theta = 0.5f * PI * u.y;
			else if (_test_scheme == 1) theta = acosf(u.y);
			else if (_test_scheme == 2) theta = acosf(sqrt(u.y));
			
			float cos_theta = 0;
			float sin_theta = 0;
			if (_test_scheme < 3) {
				sin_theta = sin(theta);
				cos_theta = cos(theta);
			}
			else {
				float a = _roughness*_roughness;

				cos_theta = sqrtf( (1.0f - u.y) / (1.0f + (a*a - 1.0f) * u.y) );
				sin_theta = sqrtf( 1.0f - cos_theta * cos_theta );
			}

					
			return float3(sin_theta * cos(phi),
			              sin_theta * sin(phi),
			              cos_theta);
		};

		float3 base = float3(-1000, 0, 0);
		for (int i=0; i<_test_samples; ++i) {
			float3 point = sample_distribution(i, _test_samples);

			g_dbgdraw.point(base + point*10, _test_size, lrgba(1,0,0,1));
		}
	}
#endif
};

// TODO: Debug why cascade 0 is not working???
struct DirectionalCascadedShadowmap {
	SERIALIZE(DirectionalCascadedShadowmap, shadow_res, cascades, cascade_factor, size, depth_range, bias_fac_world, bias_max_world)

	// (square) pixel resultion of each shadowmap cascade
	int shadow_res = 2048;
	// number of cascades
	int cascades = 4;

	float cascade_factor = 3.0f;

	// worldspace size of first cascade (width and height, this determines shadow resolution on surfaces)
	float size = 512;
	// worldspace length of cascade (what range the depth gets mapped to, this determines depth artefacts)
	float depth_range = 700;

	// shadow bias parameters
	float bias_fac_world = 2.0f;
	float bias_max_world = 10.0f;

	// computed values for shader
	float texel_size;
	float bias_fac;
	float bias_max;

	static constexpr GLenum depth_format = GL_DEPTH_COMPONENT16;

	class Textures {
		GLuint tex = 0;
	public:
		MOVE_ONLY_CLASS_MEMBER(Textures, tex);
	
		Textures () {} // not allocated
		Textures (std::string_view label, int2 size, int count, GLenum format) {
			glGenTextures(1, &tex);

			glBindTexture(GL_TEXTURE_2D_ARRAY, tex);
			OGL_DBG_LABEL(GL_TEXTURE, tex, label);

			glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, format, size.x, size.y, count);
			glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_BASE_LEVEL, 0);
			glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAX_LEVEL, 0);
			glTexParameterf(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameterf(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

			glBindTexture(GL_TEXTURE_2D_ARRAY, 0);
		}
		~Textures () {
			if (tex) glDeleteTextures(1, &tex);
		}

		operator GLuint () const { return tex; }
	};

	Textures shadow_tex;
	
	// without shadow sampler (filter depth, then test)
	Sampler sampler = make_sampler("DirectionalShadowmap.sampler", FILTER_BILINEAR, GL_CLAMP_TO_EDGE);
	// with shadow sampler (test texels, then filter)
	Sampler sampler2 = make_sampler("DirectionalShadowmap.sampler", FILTER_BILINEAR, GL_CLAMP_TO_EDGE);

	// TODO: might be able to use layered FBOs? what's the advantage?
	std::vector<Fbo> fbos;
	
	// needed later during lighting
	View3D view_casc0;

	bool tex_changed = true;

	void imgui () {
		if (!ImGui::TreeNode("DirectionalShadowmap")) return;

		tex_changed = ImGui::InputInt("shadow_res", &shadow_res) || tex_changed;
		tex_changed = ImGui::InputInt("cascades", &cascades) || tex_changed;
		shadow_res = clamp(shadow_res, 1, 1024*16);
		cascades = clamp(cascades, 1, 16);
		
		ImGui::DragFloat("cascade_factor", &cascade_factor, 0.1f, 1, 8);

		ImGui::DragFloat("size", &size, 0.1f, 0, 1024*16);
		ImGui::DragFloat("depth_range", &depth_range, 0.1f, 0, 1024*16);

		ImGui::DragFloat("bias_fac", &bias_fac_world, 0.1f);
		ImGui::DragFloat("bias_max", &bias_max_world, 0.1f);

		ImGui::Text("res: %.3f m\nbias_fac: %.4f m (%.5f depth)\nbias_max: %.4f m (%.5f depth)",
			texel_size, bias_fac_world * texel_size, bias_fac, bias_max_world * texel_size, bias_max);

		ImGui::TreePop();
	}

	void resize (int2 tex_res) {
		glActiveTexture(GL_TEXTURE0);

		shadow_tex = Textures("DirectionalShadowmap.depth", tex_res, cascades, depth_format);
		
		glSamplerParameteri(sampler2, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
		glSamplerParameteri(sampler2, GL_TEXTURE_COMPARE_FUNC, GL_GREATER);

		fbos.resize(cascades);
		
		for (int i=0; i<cascades; ++i) {
			fbos[i] = Fbo("DirectionalShadowmap.fbo");
			glBindFramebuffer(GL_FRAMEBUFFER, fbos[i]);

			glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, shadow_tex, 0, i);

			glDrawBuffers(0, nullptr);

			GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
			if (status != GL_FRAMEBUFFER_COMPLETE) {
				fprintf(stderr, "glCheckFramebufferStatus: %x\n", status);
			}
		}

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}

	void update () {
		// bias at 45deg should be size of shadowmap texel in world space because with flat surface can sample at most that amount wrong
		texel_size = size / (float)shadow_res;

		bias_fac = bias_fac_world * texel_size / depth_range;
		bias_max = bias_max_world * texel_size / depth_range;


		if (!tex_changed) return;
		tex_changed = false;

		resize(int2(shadow_res,shadow_res));
	}

	void prepare_uniforms_and_textures (Shader* shad, TextureBinds& textures) {
		shad->set_uniform("shadowmap_mat", view_casc0.world2clip);
		shad->set_uniform("shadowmap_dir", (float3x3)view_casc0.cam2world * float3(0,0,-1));
		shad->set_uniform("shadowmap_bias_fac", bias_fac);
		shad->set_uniform("shadowmap_bias_max", bias_max);
		shad->set_uniform("shadowmap_cascade_factor", cascade_factor);

		textures += { "shadowmap",  { GL_TEXTURE_2D_ARRAY, shadow_tex }, sampler };
		textures += { "shadowmap2", { GL_TEXTURE_2D_ARRAY, shadow_tex }, sampler2 };
	}

	template <typename FUNC>
	void draw_cascades (View3D& view, GameTime::SkyConfig& sky, StateManager& state, FUNC draw_scene) {
		
		float casc_size = size;
		float casc_depth_range = depth_range;

		for (int i=0; i<cascades; ++i) {
			auto& cascade_fbo = fbos[i];

			ZoneScopedN("draw cascade");
			OGL_TRACE("draw cascade");

			glBindFramebuffer(GL_FRAMEBUFFER, cascade_fbo);
			glViewport(0, 0, shadow_res, shadow_res);
			
			PipelineState s; // Set state (glDepthMask) so glClear actully bothers to do anything!
			state.set_no_override(s);
			glClear(GL_DEPTH_BUFFER_BIT);

			//float3 center = float3(size * 0.5f, size * 0.5f, 0.0f);
			float3 center = view.cam_pos;
			center.z = 0; // force shadowmap center to world plane to avoid depth range running out
			
			float3x4 world2sun = sky.world2sun * translate(-center);
			float3x4 sun2world = translate(center) * sky.sun2world;

			View3D view = ortho_view(casc_size, casc_size, -casc_depth_range*0.5f, +casc_depth_range*0.5f,
				world2sun, sun2world, (float)shadow_res);
			
			if (i == 0)
				view_casc0 = view;

			casc_size *= cascade_factor;
			casc_depth_range *= cascade_factor;

			draw_scene(view, cascade_fbo, shadow_res, depth_format);
		}
	}
};

struct DecalRenderer {
	Shader* shad  = g_shaders.compile("decals");

	struct Vertex {
		float3 pos;

		VERTEX_CONFIG(
			ATTRIB(FLT,3, Vertex, pos),
		)
	};
	struct Instance {
		float3 pos;
		float  rot;
		float3 size;
		int    tex_id;
		float2 uv_scale;
		float4 col;

		VERTEX_CONFIG(
			ATTRIB(FLT,3, Instance, pos),
			ATTRIB(FLT,1, Instance, rot),
			ATTRIB(FLT,3, Instance, size),
			ATTRIB(INT,1, Instance, tex_id),
			ATTRIB(FLT,2, Instance, uv_scale),
			ATTRIB(FLT,4, Instance, col),
		)
	};
	
	// Decals that simply blend over gbuf color, normals etc
	
	VertexBufferInstancedI vbo = vertex_buffer_instancedI<Vertex, Instance>("DecalRenderer.vbo");

	DecalRenderer () {
		using namespace render::shapes;
		vbo.upload_mesh(CUBE_CENTERED_VERTICES, ARRLEN(CUBE_CENTERED_VERTICES), CUBE_INDICES, ARRLEN(CUBE_INDICES));
	}
	
	GLsizei instance_count = 0;

	void upload (std::vector<Instance>& instances) {
		vbo.stream_instances(instances);
		instance_count = (GLsizei)instances.size();
	}
	
	// requires gbuf_depth to correctly intersect existing geomertry and project decals
	void render (StateManager& state, Gbuffer& gbuf, Textures& texs) {
		ZoneScoped;
		OGL_TRACE("DecalRenderer");

		if (shad->prog) {
			glUseProgram(shad->prog);

			state.bind_textures(shad, {
				{ "gbuf_depth", { GL_TEXTURE_2D, gbuf.depth }, gbuf.sampler }, // allowed to read depth because we are not writing to it

				{ "cracks", *texs.cracks, texs.sampler_normal },
			});

			PipelineState s;
			// depth test with DEPTH_BEHIND, causing backfaces that insersect with gbuffer to be drawn
			// unfortunately, this won't exclude volumes completely occluded by a wall, but this is better than no depth testing at all
			// (The shader still manually intersect the volume to correctly exclude pixels)
			s.depth_test = true;
			s.depth_func = DEPTH_BEHIND;
			s.depth_write = false; // depth write off!
			s.cull_face = true;
			s.front_face = CULL_FRONT; // draw backfaces to avoid camera in volume
			s.blend_enable = true; // default blend mode (standard alpha)
			state.set(s);

			if (instance_count > 0) {
				glBindVertexArray(vbo.vao);
				glDrawElementsInstanced(GL_TRIANGLES, ARRLEN(render::shapes::CUBE_INDICES), GL_UNSIGNED_SHORT, (void*)0, instance_count);
			}
		}

		glBindVertexArray(0);
	}
};

struct ClippingRenderer {
	Shader* shad  = g_shaders.compile("clipping");

	struct Vertex {
		float3 pos;
		float3 norm;

		VERTEX_CONFIG(
			ATTRIB(FLT,3, Vertex, pos),
			ATTRIB(FLT,3, Vertex, norm),
		)
	};
	struct Instance {
		float3 pos;
		float  rot;
		float3 size;

		VERTEX_CONFIG(
			ATTRIB(FLT,3, Instance, pos),
			ATTRIB(FLT,1, Instance, rot),
			ATTRIB(FLT,3, Instance, size),
		)
	};
	
	VertexBufferInstancedI vbo = vertex_buffer_instancedI<Vertex, Instance>("ClippingRenderer.vbo");

	ClippingRenderer () {
		SimpleMesh<VertexPN> mesh;
		if (!assimp::load_simple("assets/misc/cube.fbx", &mesh)) {
			assert(false);
		}
		
		vbo.upload_mesh(mesh.vertices, mesh.indices);
		index_count = (GLsizei)mesh.indices.size();
	}
	
	GLsizei index_count = 0;
	GLsizei instance_count = 0;

	void upload (std::vector<Instance>& instances) {
		vbo.stream_instances(instances);
		instance_count = (GLsizei)instances.size();
	}

	int2 resolution = -1;
	Render_Texture tmp_copy_tex;
	Sampler depth_sampler = make_sampler("gbuf_sampler", FILTER_NEAREST, GL_CLAMP_TO_EDGE);
	
	void render (StateManager& state, Fbo& depth_fbo, int2 res, GLenum depth_format, Textures& texs, bool shadow_pass=false) {
		ZoneScoped;
		OGL_TRACE("ClippingRenderer");

		if (resolution != res) {
			tmp_copy_tex = Render_Texture("gbuf.depth", res, depth_format);
			resolution = res;
		}

		// Need to copy depth buffer because we can't write to it and read it at the same time, no idea if there is a good way around this
		// maybe rendering into stencil buffer and doing something with that
		// copies from current FBO, no idea how it knows which channel to copy...
		glBindTexture(GL_TEXTURE_2D, tmp_copy_tex);
		glCopyTexSubImage2D(GL_TEXTURE_2D,0, 0,0, 0,0, res.x, res.y);
		glBindTexture(GL_TEXTURE_2D, 0);

		if (shad->prog) {
			glUseProgram(shad->prog);

			state.bind_textures(shad, {
				{ "gbuf_depth", { GL_TEXTURE_2D, tmp_copy_tex }, depth_sampler },
				//{ "test_tex", texs.terrain_diffuse, texs.sampler_normal },
			});

			PipelineState s;
			s.depth_test = true;
			s.depth_func = DEPTH_BEHIND;
			s.depth_write = true;
			s.cull_face = true;
			s.front_face = CULL_FRONT;
			if (shadow_pass) {
				s.depth_clamp = true;
			}
			state.set(s);

			if (instance_count > 0) {
				glBindVertexArray(vbo.vao);
				glDrawElementsInstanced(GL_TRIANGLES, index_count, GL_UNSIGNED_SHORT, (void*)0, instance_count);
			}
		}

		glInvalidateTexImage(tmp_copy_tex, 0); // Is this good practice?

		glBindVertexArray(0);
	}
};

struct DefferedPointLightRenderer {
	SERIALIZE(DefferedPointLightRenderer, enable)

	typedef typename VertexPos3 vert_t;
	typedef typename uint32_t   idx_t;

	Shader* shad = g_shaders.compile("point_lights");

	bool enable = true;

	struct LightInstance {
		//int    type;
		float3 pos;
		float  radius;
		float3 dir;
		float2 cone;
		float3 col; // col * strength

		VERTEX_CONFIG(
			ATTRIB(FLT,3, LightInstance, pos),
			ATTRIB(FLT,1, LightInstance, radius),
			ATTRIB(FLT,3, LightInstance, dir),
			ATTRIB(FLT,2, LightInstance, cone),
			ATTRIB(FLT,3, LightInstance, col),
		)
	};

	VertexBufferInstancedI vbo = vertex_buffer_instancedI<vert_t, LightInstance>("point_lights");

	GLsizei index_count = 0;
	GLsizei instance_count = 0;

	DefferedPointLightRenderer () {
		SimpleMesh<VertexPos3> mesh;
		if (!assimp::load_simple("assets/misc/ico_sphere.fbx", &mesh)) {
			assert(false);
		}

		vbo.upload_mesh(mesh.vertices, mesh.indices);
		index_count = (GLsizei)mesh.indices.size();
	}

	void imgui () {
		ImGui::Checkbox("enable lights", &enable);
	}

	void update_instances (std::vector<LightInstance> const& instances) {
		ZoneScoped;
		OGL_TRACE("update lights");

		vbo.stream_instances(instances);

		instance_count = (GLsizei)instances.size();
	}

	void draw (StateManager& state, Gbuffer& gbuf, PBR_Render& pbr) {
		if (enable && shad->prog) {
			ZoneScoped;
			OGL_TRACE("draw lights");

			// additive light blending
			PipelineState s;
			s.depth_test = false;
			//s.depth_test = true;
			//s.depth_func = DEPTH_BEHIND;
			s.depth_write = false;

			//s.cull_face = false; // need to draw lights even when inside the light radius sphere
			s.cull_face = true;
			s.front_face = CULL_FRONT; // draw backfaces to avoid camera in light volume problem

			s.blend_enable = true; // additive light shading
			s.blend_func.dfactor = GL_ONE;
			s.blend_func.sfactor = GL_ONE;
			s.blend_func.equation = GL_FUNC_ADD;

			state.set(s);

			glUseProgram(shad->prog);
			
			TextureBinds tex;
			tex += gbuf.textures();
			pbr.prepare_uniforms_and_textures(shad, tex);
			state.bind_textures(shad, tex);

			glBindVertexArray(vbo.vao);
			glDrawElementsInstanced(GL_TRIANGLES, index_count, GL_UNSIGNED_SHORT, (void*)0, instance_count);
			glBindVertexArray(0);
		}
	}
};

// framebuffer for rendering at different resolution and to make sure we get float buffers
struct RenderPasses {
	SERIALIZE(RenderPasses, renderscale, shadowmap, exposure)

	Gbuffer gbuf;

	// Need to include gbuf depth buffer in lighting fbo because point light require depth testing (but not depth writing)
	// at the same time we can't apply defferred point lights to the gbuf itself, since albedo need to stay around, so we need this seperate render target
	// attaching the existing depth to a second FBO should be the correct solution
	struct LightingFbo {
		MOVE_ONLY_CLASS(LightingFbo); // No move implemented for now
	public:

		static void swap (LightingFbo& l, LightingFbo& r) {
			std::swap(l.fbo, r.fbo);
			std::swap(l.col, r.col);
		}

		Fbo fbo = {};
		Render_Texture col = {};

		LightingFbo () {}
		LightingFbo (std::string_view label, int2 size, GLenum color_format, Render_Texture& depth, bool mips=false) {
			GLint levels = mips ? calc_mipmaps(size.x, size.y) : 1;

			std::string lbl = (std::string)label;

			col = Render_Texture(lbl+".col", size, color_format, levels);

			{
				fbo = Fbo(lbl+".fbo");
				glBindFramebuffer(GL_FRAMEBUFFER, fbo);

				glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, col, 0);
				glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth, 0);

				GLuint bufs[] = { GL_COLOR_ATTACHMENT0 };
				glDrawBuffers(ARRLEN(bufs), bufs);
		
				GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
				if (status != GL_FRAMEBUFFER_COMPLETE) {
					fprintf(stderr, "glCheckFramebufferStatus: %x\n", status);
				}
			}

			glBindFramebuffer(GL_FRAMEBUFFER, 0);
			glBindTexture(GL_TEXTURE_2D, 0);
		}

	};
	LightingFbo lighting_fbo;
	
	DirectionalCascadedShadowmap shadowmap;

	render::RenderScale renderscale;
	
	Sampler renderscale_sampler         = make_sampler("renderscale_sampler", FILTER_MIPMAPPED, GL_CLAMP_TO_EDGE);
	Sampler renderscale_sampler_nearest = make_sampler("renderscale_sampler_nearest", FILTER_NEAREST, GL_CLAMP_TO_EDGE);
	
	Sampler& get_renderscale_sampler () {
		return renderscale.nearest ? renderscale_sampler_nearest : renderscale_sampler;
	}

	PBR_Render pbr;
	
	Shader* shad_fullscreen_lighting = g_shaders.compile("fullscreen_lighting");
	Shader* shad_postprocess = g_shaders.compile("postprocess");
	
	float exposure = 1.0f;
	
	void imgui () {
		renderscale.imgui();
		shadowmap.imgui();
		pbr.imgui();

		if (ImGui::TreeNode("Postprocessing")) {
			ImGui::Text("For how many Lux is the camera exposure set?");
			// Note: slider is backwards because lux is the useful unit, but 'more' exposure means exposing for less lux
			ImGui::SliderFloat("exposure", &exposure, 1000000.0f, 0.0001f, "%12.4f", ImGuiSliderFlags_Logarithmic);
			ImGui::TreePop();
		}
	}
	
	void update (StateManager& state, Textures& texs, int2 window_size) {
		ZoneScoped;

		shadowmap.update();
		pbr.update(state, texs);

		if (renderscale.update(window_size)) {
			gbuf.resize(renderscale.size);
			lighting_fbo = LightingFbo("lighting_fbo", renderscale.size, GL_RGB16F, gbuf.depth, true);
		}
	}

	void begin_geometry_pass (StateManager& state) {
		glBindFramebuffer(GL_FRAMEBUFFER, gbuf.fbo);
		glViewport(0, 0, renderscale.size.x, renderscale.size.y);
		
		PipelineState s;
		state.set(s);

		glClearColor(0.01f, 0.02f, 0.03f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//glClear(GL_DEPTH_BUFFER_BIT);
	}
	
	void begin_lighting_pass () {
		glBindFramebuffer(GL_FRAMEBUFFER, lighting_fbo.fbo);
		glViewport(0, 0, renderscale.size.x, renderscale.size.y);
	}

	void fullscreen_lighting_pass (StateManager& state, Textures& texs, DefferedPointLightRenderer& light_renderer) {
		if (shad_fullscreen_lighting->prog) {
			ZoneScoped;
			OGL_TRACE("fullscreen_lighting");

			glUseProgram(shad_fullscreen_lighting->prog);

			TextureBinds tex;
			tex += gbuf.textures();
			pbr.prepare_uniforms_and_textures(shad_fullscreen_lighting, tex);

			tex += { "clouds", *texs.clouds, texs.sampler_normal };
			tex += { "night_sky", texs.night_sky, texs.sampler_cubemap };
			tex += { "moon", *texs.moon };
			tex += { "moon_nrm", *texs.moon_nrm };
			tex += { "grid_tex", *texs.grid, texs.sampler_normal };
			tex += { "contours_tex", *texs.contours, texs.sampler_normal };

			shadowmap.prepare_uniforms_and_textures(shad_fullscreen_lighting, tex);
			
			state.bind_textures(shad_fullscreen_lighting, tex);
			draw_fullscreen_triangle(state);
		}
		
		light_renderer.draw(state, gbuf, pbr);
	}
	void end_lighting_pass () {
		ZoneScoped;
		OGL_TRACE("lighting_fbo gen mipmaps");

		glBindTexture(GL_TEXTURE_2D, lighting_fbo.col);
		glGenerateMipmap(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	void postprocess (StateManager& state, int2 window_size) {
		ZoneScoped;
		OGL_TRACE("postprocess");

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glViewport(0, 0, window_size.x, window_size.y);
		
		if (shad_postprocess->prog) {
			glUseProgram(shad_postprocess->prog);
				
			shad_postprocess->set_uniform("exposure", 1.0f / exposure); // exposure in lux to map lux into 0-1 requires division

			state.bind_textures(shad_postprocess, {
				{ "lighting_fbo", { GL_TEXTURE_2D, lighting_fbo.col }, get_renderscale_sampler() }
			});
			draw_fullscreen_triangle(state);
		}
	}
};

} // namespace ogl
