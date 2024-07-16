#pragma once
#include "opengl.hpp"
#include "agnostic_render.hpp"
#include "bindless_textures.hpp"

namespace ogl {
	
struct Textures {
	// TODO: need some sort of texture array or atlas system! (atlas sucks tho)
	// -> find out if there is a modern system for single drawcall many textures (of differing sizes)
	// because it would be wierd that you can go all out with indirect instanced drawing, yet have to adjust your textures
	// just to be able to use them with one texture array

	BindlessTextureManager bindless_textures;

	struct DiffNorm {
		const char* diffuse;
		const char* normal;
	};

	Sampler sampler_normal = make_sampler("sampler_normal", FILTER_MIPMAPPED, GL_REPEAT, true);

	const char* turn_arrows[7] = {
		"misc/turn_arrow_R.png"  ,
		"misc/turn_arrow_S.png"  ,
		"misc/turn_arrow_SR.png" ,
		"misc/turn_arrow_L.png"  ,
		"misc/turn_arrow_LR.png" ,
		"misc/turn_arrow_LS.png" ,
		"misc/turn_arrow_LSR.png",
	};

	DiffNorm asphalt  = { "misc/street/pebbled_asphalt_albedo.png",   "misc/street/pebbled_asphalt_Normal-ogl.png" };
	DiffNorm pavement = { "misc/street/Flooring_Stone_001_COLOR.png", "misc/street/Flooring_Stone_001_NRM.png" };
	DiffNorm curb     = { "misc/curb_Diff2.png",                      "misc/curb_Norm.png" };
	

	Texture2D clouds, grid, contours;
	Texture2D terrain_diffuse;
	
	Texture2D house_diff;
	Texture2D streetlight_diff;

	Texture2D cracks;
	
	struct HeightmapTextures {
		Texture2D inner, outer;

		Sampler sampler = make_sampler("sampler_heightmap", FILTER_BILINEAR, GL_CLAMP_TO_EDGE);
	
		void upload (Heightmap const& heightmap) {
			inner = upload_image<uint16_t>("heightmap.inner", heightmap.inner, false);
			outer = upload_image<uint16_t>("heightmap.outer", heightmap.outer, false);
		}
		
		TextureBinds textures () {
			return {{
				{"heightmap_inner", inner, sampler},
				{"heightmap_outer", outer, sampler},
			}};
		}
	};

	HeightmapTextures heightmap;

	template <typename T>
	static Texture2D upload_image (std::string_view gl_label, Image<T> const& img, bool mips=true) {
		ZoneScoped;

		Texture2D tex = {gl_label};
		upload_image2D<T>(tex, img, mips);
		return tex;
	}
	template <typename T>
	static Texture2D load_texture (std::string_view gl_label, const char* filepath, bool mips=true) {
		ZoneScoped;
		
		printf("loading texture \"%s\"...\n", filepath);

		Texture2D tex = {gl_label};
		if (!upload_texture2D<T>(tex, prints("assets/%s", filepath).c_str(), mips))
			assert(false);
		return tex;
	}

	template <typename T>
	void load_bindless (const char* filename) {
		ZoneScoped;

		bindless_textures.load_texture<T>(filename);
	}
	
	template <typename T>
	void load_bindless (DiffNorm& df) {
		ZoneScoped;

		bindless_textures.load_texture<T>(df.diffuse);
		bindless_textures.load_texture<T>(df.normal);
	}

	void reload_all () {
		ZoneScoped;

		// TODO: don't hardcode these, rather put them into assets and have used assets trigger load of their textures
		
		bindless_textures.clear();


		clouds = load_texture<srgba8>("clouds", "skybox/clouds.png");
		grid = load_texture<srgba8>("grid", "misc/grid2.png");
		contours = load_texture<srgba8>("contours", "misc/contours.png");
		terrain_diffuse = load_texture<srgb8>("terrain_diffuse", "misc/Rock_Moss_001_SD/Rock_Moss_001_basecolor.jpg");
	
		house_diff = load_texture<srgb8>("house_Diff", "buildings/house.png");
		streetlight_diff = load_texture<srgb8>("streetlight_Diff", "props/streetlight_Diff.png");

		cracks = load_texture<srgb8>("cracks", "misc/cracks.png"); // TODO: support single channel


		load_bindless<srgba8>("misc/line.png"          );
		load_bindless<srgba8>("misc/stripe.png"        );
		load_bindless<srgba8>("misc/shark_teeth.png"   );
		
		for (auto& fn : turn_arrows)
			load_bindless<srgba8>(fn);
		
		load_bindless<srgb8>(asphalt);
		load_bindless<srgb8>(pavement);
		load_bindless<srgb8>(curb);
		
		// TODO: make dynamic so that any texture from json works
		load_bindless<srgb8>("vehicles/car.diff.png");
		load_bindless<srgb8>("vehicles/car.TR.png");

		load_bindless<srgb8>("vehicles/bus.diff.png");
		load_bindless<srgb8>("vehicles/bus.TR.png");
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

	// only depth -> reconstruct position  float32 format for resonable infinite far plane (float16 only works with ~1m near plane)
	static constexpr GLenum depth_format = GL_DEPTH_COMPONENT32F; // GL_R32F causes GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT
	// rgb albedo (emmisive is simply very bright)
	// might be able to afford using GL_RGB8 since it's mostly just albedo from textures
	static constexpr GLenum col_format   = GL_RGB16F; // GL_RGB16F GL_RGB8
	// rgb normal
	// GL_RGB8 requires encoding normals, might be better to have camera space normals without z
	// seems like 8 bits might be to little for normals
	static constexpr GLenum norm_format  = GL_RGB16F; // GL_RGB16F
	// PBR roughness, metallic
	static constexpr GLenum pbr_format   = GL_RG8;

	Render_Texture depth  = {};
	Render_Texture col    = {};
	Render_Texture norm   = {};
	Render_Texture pbr    = {};

	Sampler sampler = make_sampler("gbuf_sampler", FILTER_NEAREST, GL_CLAMP_TO_EDGE);

	void resize (int2 size) {
		glActiveTexture(GL_TEXTURE0);
		
		depth  = Render_Texture("gbuf.depth", size, depth_format);
		col    = Render_Texture("gbuf.col",   size, col_format);
		norm   = Render_Texture("gbuf.norm",  size, norm_format);
		pbr    = Render_Texture("gbuf.pbr",   size, pbr_format);
		
		fbo = Fbo("gbuf.fbo");
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,  GL_TEXTURE_2D, depth, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, col, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, norm, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, pbr, 0);
		
		// needed so layout(location = x) out vec3 frag_x; works
		GLuint bufs[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2 };
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
			{ "gbuf_norm",  { GL_TEXTURE_2D, norm  }, sampler },
			{ "gbuf_pbr",   { GL_TEXTURE_2D, pbr   }, sampler },
		}};
	}
};

struct PBR_Render {
	Shader* shad_integrate_brdf = g_shaders.compile("pbr_integrate_brdf");
	Shader* shad_prefilter_env  = g_shaders.compile("pbr_prefilter_env");

	Texture2D brdf_LUT;
	int brdf_LUT_res = 256; // Seems to be enough
	bool recreate_brdf = true;
	
	TextureCubemap env_map;
	int env_res = 512;
	int env_mips = 7;
	// adjust roughness with  roughness_mip = pow(roughness, env_roughness_curve)
	// to squeeze more of the low roughness into the lower mips, so we waste less blurry high roughness on the high-res mips
	// Note: technically makes interpolation non-linear (wrong), but probably is still more correct than a too blurry mip
	float env_roughness_curve = 0.6f;
	bool recreate_env_map = true;
	bool redraw_env_map = true;

	std::vector<std::array<Fbo, 6>> env_fbos;

	void imgui () {
		if (ImGui::TreeNode("PBR")) {
			recreate_brdf = ImGui::SliderInt("brdf_LUT_res", &brdf_LUT_res, 1, 1024, "%d", ImGuiSliderFlags_Logarithmic) || recreate_brdf;

			recreate_env_map = ImGui::SliderInt("env_res", &env_res, 1, 2048, "%d", ImGuiSliderFlags_Logarithmic) || recreate_env_map;
			
			int mips = calc_mipmaps(env_res, env_res);
			recreate_env_map = ImGui::SliderInt("env_mips", &env_mips, 1, mips, "%d") || recreate_env_map;
			
			int lowest_res = env_res >> (env_mips-1);
			ImGui::Text("Env map lowest mip: %dx%d pixels", lowest_res, lowest_res);

			ImGui::TreePop();
		}
	}

	template <typename FUNC>
	static auto draw_into_texture_with_temp_fbo (GLuint textarget, GLuint tex, int2 size, int mip, FUNC draw) {
		auto fbo = Fbo("tmp_fbo");
		// assume texture bound
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);

		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, textarget, tex, mip);
		GLuint bufs[] = { GL_COLOR_ATTACHMENT0 };
		glDrawBuffers(ARRLEN(bufs), bufs);
		
		GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
		if (status != GL_FRAMEBUFFER_COMPLETE) {
			//fprintf(stderr, "glCheckFramebufferStatus: %x\n", status);
		}
		
		glViewport(0, 0, size.x, size.y);

		draw();
		
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}
	static Fbo create_fbo (GLuint textarget, GLuint tex, int mip) {
		auto fbo = Fbo("PBR.env_fbo");
		// assume texture bound
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);

		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, textarget, tex, mip);
		GLuint bufs[] = { GL_COLOR_ATTACHMENT0 };
		glDrawBuffers(ARRLEN(bufs), bufs);
		
		GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
		if (status != GL_FRAMEBUFFER_COMPLETE) {
			//fprintf(stderr, "glCheckFramebufferStatus: %x\n", status);
		}

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		return fbo;
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
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		draw_into_texture_with_temp_fbo(GL_TEXTURE_2D, brdf_LUT, brdf_LUT_res, 0, [&] () {
			// clear for good measure
			glClearColor(0,0,0,0);
			glClear(GL_COLOR_BUFFER_BIT);
				
			glUseProgram(shad_integrate_brdf->prog);
			shad_integrate_brdf->set_uniform("resolution", float2((float)brdf_LUT_res));
				
			state.bind_textures(shad_integrate_brdf, {});
			draw_fullscreen_triangle(state);
		});

		glBindTexture(GL_TEXTURE_2D, 0);
	}
	
	void create_env_map () {
		ZoneScoped;
		OGL_TRACE("create_env_map");
			
		// recreate texture
		env_map = {"PBR.env_map"};
		glBindTexture(GL_TEXTURE_CUBE_MAP, env_map);

		// allocates all faces in a single call
		env_mips = min(env_mips, calc_mipmaps(env_res, env_res));
		glTexStorage2D(GL_TEXTURE_CUBE_MAP, env_mips, GL_RGB16F, env_res, env_res);

		env_fbos.resize(env_mips);
		for (int mip=0; mip<env_mips; ++mip) {
			for (int face=0; face<6; ++face) {
				env_fbos[mip][face] = create_fbo(GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, env_map, mip);
			}
		}

		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_BASE_LEVEL, 0);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAX_LEVEL, env_mips-1);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
			
		glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
	}
	void draw_env_map (StateManager& state, Textures& texs) {
		ZoneScoped;
		OGL_TRACE("draw_env_map");

		glUseProgram(shad_prefilter_env->prog);
		state.bind_textures(shad_prefilter_env, {
			{ "clouds", texs.clouds, texs.sampler_normal }
		});

		int2 res = env_res;
		for (int mip=0; mip<env_mips; ++mip) {
			for (int face=0; face<6; ++face) {
				glViewport(0, 0, res.x, res.y);
				glBindFramebuffer(GL_FRAMEBUFFER, env_fbos[mip][face]);

				// clear for good measure
				glClearColor(0,0,0,0);
				glClear(GL_COLOR_BUFFER_BIT);
					
				float roughness = 0.0f;
				if (env_mips > 1) {
					float t = (float)mip / (float)(env_mips-1);
					roughness = powf(t, 1.0f / env_roughness_curve);
				}

				shad_prefilter_env->set_uniform("resolution", (float2)res);
				shad_prefilter_env->set_uniform("face_i", face);
				shad_prefilter_env->set_uniform("mip_i", mip);
				shad_prefilter_env->set_uniform("roughness", roughness);
				
				draw_fullscreen_triangle(state);
			}
			res = max(res / 2, 1);
		}
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		
		// Maybe actually generate mips for the raw env map and put prefiltered in a seperate cubemap?
		// mips could help reduce the sample counts
		//glBindTexture(GL_TEXTURE_CUBE_MAP, env_map);
		//glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
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
		tex += { "pbr_env_map", env_map };
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

		shadow_tex = Textures("DirectionalShadowmap.depth", tex_res, cascades, GL_DEPTH_COMPONENT16);
		
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

		textures += { "shadowmap", { GL_TEXTURE_2D_ARRAY, shadow_tex }, sampler };
		textures += { "shadowmap2", { GL_TEXTURE_2D_ARRAY, shadow_tex }, sampler2 };
	}

	template <typename FUNC>
	void draw_cascades (App& app, View3D& view, StateManager& state, FUNC draw_scene) {
		
		float casc_size = size;
		float casc_depth_range = depth_range;

		for (int i=0; i<cascades; ++i) {
			auto& cascade_fbo = fbos[i];

			ZoneScopedN("draw cascade");
			OGL_TRACE("draw cascade");

			glBindFramebuffer(GL_FRAMEBUFFER, cascade_fbo);
			glViewport(0, 0, shadow_res, shadow_res);
		
			glClear(GL_DEPTH_BUFFER_BIT);

			//float3 center = float3(size * 0.5f, size * 0.5f, 0.0f);
			float3 center = view.cam_pos;
			center.z = 0; // force shadowmap center to world plane to avoid depth range running out
		
			float3x4 world2cam = app.time_of_day.world2sun * translate(-center);
			float3x4 cam2world = translate(center) * app.time_of_day.sun2world;

			View3D view = ortho_view(casc_size, casc_size, -casc_depth_range*0.5f, +casc_depth_range*0.5f,
				world2cam, cam2world, (float)shadow_res);
			
			if (i == 0)
				view_casc0 = view;

			casc_size *= cascade_factor;
			casc_depth_range *= cascade_factor;

			draw_scene(view);
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
	
	// TODO: instance this
	// TODO: make box shaped decals with falloff?

	// Decals that simply blend over gbuf color and normal channel
	
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
	
	void render (StateManager& state, Gbuffer& gbuf, Textures& texs) {
		ZoneScoped;
		OGL_TRACE("DecalRenderer");

		if (shad->prog) {
			glUseProgram(shad->prog);

			state.bind_textures(shad, {
				{ "gbuf_depth", { GL_TEXTURE_2D, gbuf.depth }, gbuf.sampler },

				{ "cracks", texs.cracks, texs.sampler_normal },
			});

			PipelineState s;
			s.depth_test = false; // don't depth test or backface won't be drawn
			s.depth_write = false;

			s.cull_face = true;
			s.front_face = CULL_FRONT; // draw backfaces to avoid camera in volume

			s.blend_enable = true;
			state.set(s);

			if (instance_count > 0) {
				glBindVertexArray(vbo.vao);
				glDrawElementsInstanced(GL_TRIANGLES, ARRLEN(render::shapes::CUBE_INDICES), GL_UNSIGNED_SHORT, (void*)0, instance_count);
			}
		}

		glBindVertexArray(0);
	}
};

struct DefferedPointLightRenderer {
	typedef typename VertexPos3 vert_t;
	typedef typename uint32_t   idx_t;

	Shader* shad = g_shaders.compile("point_lights");

	struct LightInstance {
		//int    type;
		float3 pos;
		float  radius;
		float3 col;

		VERTEX_CONFIG(
			ATTRIB(FLT,3, LightInstance, pos),
			ATTRIB(FLT,1, LightInstance, radius),
			ATTRIB(FLT,3, LightInstance, col),
		)
	};

	VertexBufferInstancedI vbo = vertex_buffer_instancedI<vert_t, LightInstance>("point_lights");

	GLsizei index_count = 0;
	GLsizei instance_count = 0;

	DefferedPointLightRenderer () {
		SimpleMesh mesh;
		if (!assimp::load_simple("assets/misc/ico_sphere.fbx", &mesh)) {
			assert(false);
		}

		vbo.upload_mesh(mesh.vertices, mesh.indices);
		index_count = (GLsizei)mesh.indices.size();
	}

	static void imgui () {

	}

	template <typename FUNC>
	void update_instances (FUNC get_instances) {
		ZoneScoped;
		OGL_TRACE("update lights");

		auto instances = get_instances();
		vbo.stream_instances(instances);

		instance_count = (GLsizei)instances.size();
	}

	void draw (StateManager& state, Gbuffer& gbuf, PBR_Render& pbr) {
		if (shad->prog) {
			ZoneScoped;
			OGL_TRACE("draw lights");

			// additive light blending
			PipelineState s;
			//s.depth_test = true; // draw only surfaces with are in light radius
			s.depth_test = false; // don't depth test as backface won't be drawn
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
			ImGui::SliderFloat("exposure", &exposure, 0.02f, 20.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
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

			tex += { "clouds", texs.clouds, texs.sampler_normal };
			tex += { "grid_tex", texs.grid, texs.sampler_normal };
			tex += { "contours_tex", texs.contours, texs.sampler_normal };

			shadowmap.prepare_uniforms_and_textures(shad_fullscreen_lighting, tex);
			
			static float _visualize_roughness = 0;
			ImGui::SliderFloat("_visualize_roughness", &_visualize_roughness, 0, 1);
			shad_fullscreen_lighting->set_uniform("_visualize_roughness", _visualize_roughness);

			//pbr.visualize_hemisphere_sampling();

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
				
			shad_postprocess->set_uniform("exposure", exposure);

			state.bind_textures(shad_postprocess, {
				{ "lighting_fbo", { GL_TEXTURE_2D, lighting_fbo.col }, get_renderscale_sampler() }
			});
			draw_fullscreen_triangle(state);
		}
	}
};

} // namespace ogl
