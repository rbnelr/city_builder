#pragma once
#include "opengl.hpp"
#include "agnostic_render.hpp"

namespace ogl {
	
struct Textures {
	// TODO: need some sort of texture array or atlas system! (atlas sucks tho)
	// -> find out if there is a modern system for single drawcall many textures (of differing sizes)
	// because it would be wierd that you can go all out with indirect instanced drawing, yet have to adjust your textures
	// just to be able to use them with one texture array

	//Texture2D clouds = load_texture<srgba8>("clouds", "textures/clouds.png");
	Texture2D grid = load_texture<srgba8>("grid", "misc/grid2.png");
	Texture2D terrain_diffuse = load_texture<srgb8>("terrain_diffuse", "misc/Rock_Moss_001_SD/Rock_Moss_001_basecolor.jpg");
	
	//Sampler sampler_heightmap = sampler("sampler_heightmap", FILTER_BILINEAR,  GL_REPEAT);
	Sampler sampler_normal = sampler("sampler_normal", FILTER_MIPMAPPED, GL_REPEAT, true);

	Texture2D house_diffuse = load_texture<srgb8>("house_diffuse", "buildings/house.png");
	Texture2D car_diffuse = load_texture<srgb8>("car_diffuse", "cars/car.png");
	
	Texture2DArray lines = load_texture_array<srgba8>("lane_arrows", {
		"misc/line.png",
		"misc/stripe.png",
	});

	Texture2DArray turn_arrows = load_texture_array<srgba8>("lane_arrows", {
		"misc/turn_arrow_R.png",
		"misc/turn_arrow_S.png",
		"misc/turn_arrow_SR.png",
		"misc/turn_arrow_L.png",
		"misc/turn_arrow_LR.png",
		"misc/turn_arrow_LS.png",
		"misc/turn_arrow_LSR.png",
	});

	Texture2DArray surfaces_color = load_texture_array<srgba8>("surfaces_color", {
		//"misc/street/Asphalt_001_COLOR.jpg",
		//"misc/street/Asphalt_002_COLOR.jpg",
		//"misc/street/Asphalt_004_COLOR.jpg",
		"misc/street/pebbled_asphalt_albedo.png",
		"misc/street/Flooring_Stone_001_COLOR.png",
	});
	Texture2DArray surfaces_normal = load_texture_array<srgba8>("surfaces_normal", {
		//"misc/street/Asphalt_001_NORM.jpg",
		//"misc/street/Asphalt_002_NORM.jpg",
		//"misc/street/Asphalt_004_NORM.jpg",
		"misc/street/pebbled_asphalt_Normal-ogl.png",
		"misc/street/Flooring_Stone_001_NRM.png",
	});

	template <typename T>
	static Texture2D load_texture (std::string_view gl_label, const char* filepath) {
		Texture2D tex = {gl_label};
		if (!upload_texture2D<T>(tex, prints("assets/%s", filepath).c_str()))
			assert(false);
		return tex;
	}

	template <typename T>
	static Texture2DArray load_texture_array (std::string_view gl_label, std::vector<const char*> filepaths) {
		Texture2DArray tex = {gl_label};

		int count = (int)filepaths.size();
		int i = 0;
		int2 size;

		glBindTexture(GL_TEXTURE_2D_ARRAY, tex);

		for (auto path : filepaths) {
			Image<T> img;
			if (!Image<T>::load_from_file(prints("assets/%s", path).c_str(), &img)) {
				fprintf(stderr, "Error! Could not load texture \"%s\"", path);
				assert(false);
				continue;
			}

			if (i == 0) {
				size = img.size;
				glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_SRGB8_ALPHA8, size.x, size.y, count, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
			}
			assert(img.size == size);

			glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0,0,i, size.x, size.y, 1, GL_RGBA, GL_UNSIGNED_BYTE, img.pixels);
			i++;
		}
			
		glGenerateMipmap(GL_TEXTURE_2D_ARRAY);

		glBindTexture(GL_TEXTURE_2D_ARRAY, 0);
		return tex;
	}
};

struct Gbuffer {
	Fbo fbo;

	// only depth -> reconstruct position  float32 format for resonable infinite far plane (float16 only works with ~1m near plane)
	static constexpr GLenum depth_format = GL_DEPTH_COMPONENT32F; // GL_R32F causes GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT
	// rgb albedo (emmisive is simply very bright)
	static constexpr GLenum col_format   = GL_RGBA16F;
	// rgb normal  
	static constexpr GLenum norm_format  = GL_RGB16F;

	Render_Texture depth  = {};
	Render_Texture col    = {};
	Render_Texture norm   = {};

	void resize (int2 size) {
		glActiveTexture(GL_TEXTURE0);
		
		depth  = Render_Texture("gbuf.depth", size, depth_format);
		col    = Render_Texture("gbuf.col",   size, col_format);
		norm   = Render_Texture("gbuf.norm",  size, norm_format);

		glBindTexture(GL_TEXTURE_2D, 0);
		
		fbo = Fbo("gbuf.fbo");
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,  GL_TEXTURE_2D, depth, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, col, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, norm, 0);
		
		// needed so layout(location = x) out vec3 frag_x; works
		GLuint bufs[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1 };
		glDrawBuffers(ARRLEN(bufs), bufs);
		
		GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
		if (status != GL_FRAMEBUFFER_COMPLETE) {
			fprintf(stderr, "glCheckFramebufferStatus: %x\n", status);
		}
		
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}
};
struct DirectionalShadowmap {
	SERIALIZE(DirectionalShadowmap, shadow_res)

	// No cascades for now

	int shadow_res = 1024;
	bool changed = true;

	Render_Texture shadow_tex;

	Fbo fbo;

	void resize (int2 size) {
		glActiveTexture(GL_TEXTURE0);

		shadow_tex = Render_Texture("DirectionalShadowmap", size, GL_DEPTH_COMPONENT16);

		glBindTexture(GL_TEXTURE_2D, 0);

		fbo = Fbo("DirectionalShadowmap.fbo");
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);

		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadow_tex, 0);

		GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
		if (status != GL_FRAMEBUFFER_COMPLETE) {
			fprintf(stderr, "glCheckFramebufferStatus: %x\n", status);
		}

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}

	void imgui () {
		if (!ImGui::TreeNode("DirectionalShadowmap")) return;

		ImGui::InputInt("shadow_res", &shadow_res);
		shadow_res = clamp(shadow_res, 1, 1024*16);

		ImGui::TreePop();
	}

	void update () {
		if (!changed) return;

		resize(int2(shadow_res,shadow_res));
	}

	template <typename FUNC>
	void begin_draw (App& app, FUNC set_view) {
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		glViewport(0, 0, shadow_res, shadow_res);

		glClear(GL_DEPTH_BUFFER_BIT);

		//Camera
		//
		//View3D
		//
		//set_view();
	}
};

// framebuffer for rendering at different resolution and to make sure we get float buffers
struct RenderPasses {
	SERIALIZE(RenderPasses, renderscale, exposure)

	Gbuffer gbuf;
	Renderbuffer lighting_fbo;
	DirectionalShadowmap shadowmap;

	render::RenderScale renderscale;
	
	Sampler fbo_sampler         = sampler("fbo_sampler", FILTER_MIPMAPPED, GL_CLAMP_TO_EDGE);
	Sampler fbo_sampler_nearest = sampler("fbo_sampler_nearest", FILTER_NEAREST, GL_CLAMP_TO_EDGE);
	
	Sampler& get_sampler () {
		return renderscale.nearest ? fbo_sampler_nearest : fbo_sampler;
	}
	
	Shader* shad_fullscreen_lighting = g_shaders.compile("fullscreen_lighting");
	Shader* shad_postprocess = g_shaders.compile("postprocess");
	
	float exposure = 1.0f;
	
	void imgui () {
		renderscale.imgui(false);

		shadowmap.imgui();

		if (ImGui::TreeNode("Postprocessing")) {
			ImGui::SliderFloat("exposure", &exposure, 0.02f, 20.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
			ImGui::TreePop();
		}
	}
	
	void update (int2 window_size) {
		shadowmap.update();

		if (renderscale.update(window_size)) {
			gbuf.resize(renderscale.size);
			lighting_fbo = Renderbuffer("lighting_fbo", renderscale.size, GL_RGB16F, true);
		}
	}

	template <typename FUNC>
	void begin_shadow_pass (App& app, FUNC set_view) {
		shadowmap.begin_draw(app, set_view);
	}
	
	void begin_geometry_pass () {
		glBindFramebuffer(GL_FRAMEBUFFER, gbuf.fbo);
		glViewport(0, 0, renderscale.size.x, renderscale.size.y);

		glClearColor(0.01f, 0.02f, 0.03f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}
	
	void begin_lighting_pass () {
		glBindFramebuffer(GL_FRAMEBUFFER, lighting_fbo.fbo);
		glViewport(0, 0, renderscale.size.x, renderscale.size.y);
	}
	void fullscreen_lighting_pass (StateManager& state, Textures& texs) {
		if (shad_fullscreen_lighting->prog) {
			OGL_TRACE("fullscreen_lighting");
			
			glUseProgram(shad_fullscreen_lighting->prog);
			
			state.bind_textures(shad_fullscreen_lighting, {
				{ "gbuf_depth", { GL_TEXTURE_2D, gbuf.depth }, fbo_sampler_nearest },
				{ "gbuf_col",   { GL_TEXTURE_2D, gbuf.col   }, fbo_sampler_nearest },
				{ "gbuf_norm",  { GL_TEXTURE_2D, gbuf.norm  }, fbo_sampler_nearest },
				
				{"grid_tex", texs.grid, texs.sampler_normal},
			});
			draw_fullscreen_triangle(state);
		}
	}
	void end_lighting_pass () {
		OGL_TRACE("lighting_fbo gen mipmaps");
		glBindTexture(GL_TEXTURE_2D, lighting_fbo.col);
		glGenerateMipmap(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	void postprocess (StateManager& state, int2 window_size) {
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glViewport(0, 0, window_size.x, window_size.y);
		
		if (shad_postprocess->prog) {
			OGL_TRACE("postprocess");
				
			glUseProgram(shad_postprocess->prog);
				
			shad_postprocess->set_uniform("exposure", exposure);

			state.bind_textures(shad_postprocess, {
				{ "lighting_fbo", { GL_TEXTURE_2D, lighting_fbo.col }, get_sampler() }
			});
			draw_fullscreen_triangle(state);
		}
	}
};

} // namespace ogl
