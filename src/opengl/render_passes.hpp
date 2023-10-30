#pragma once
#include "opengl.hpp"
#include "agnostic_render.hpp"

namespace ogl {

struct GbufRenderbuffer {

	GLuint fbo = 0;
	GLuint col = 0;
	GLuint depth = 0;

	GbufRenderbuffer () {} // not allocated
	GbufRenderbuffer (std::string_view label, int2 size, GLenum color_format, GLenum depth_format=0, bool color_mips=false, int msaa=1) { // allocate
		GLint levels = color_mips ? calc_mipmaps(size.x, size.y) : 1;

		std::string lbl = (std::string)label;

		col = make_msaa_tex(lbl+".col", size, color_format, levels, msaa);

		if (depth_format)
			depth = make_msaa_tex(lbl+".depth", size, depth_format, levels, msaa);

		GLenum msaa_targ = msaa > 1 ? GL_TEXTURE_2D_MULTISAMPLE : GL_TEXTURE_2D;

		{
			glGenFramebuffers(1, &fbo);
			glBindFramebuffer(GL_FRAMEBUFFER, fbo);
			OGL_DBG_LABEL(GL_FRAMEBUFFER, fbo, lbl+".fbo");

			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, msaa_targ, col, 0);
			if (depth)
				glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, msaa_targ, depth, 0);

			GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
			if (status != GL_FRAMEBUFFER_COMPLETE) {
				fprintf(stderr, "glCheckFramebufferStatus: %x\n", status);
			}
		}

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glBindTexture(msaa_targ, 0);
	}
	~GbufRenderbuffer () {
		if (fbo)   glDeleteFramebuffers(1, &fbo);
		if (col)   glDeleteTextures(1, &col);
		if (depth) glDeleteTextures(1, &depth);
	}
};


struct Gbuffer {
	GLuint pos    = 0;
	GLuint col    = 0;
	GLuint norm   = 0;

	void resize (int2 size) {
		glActiveTexture(GL_TEXTURE0);

		pos    = Renderbuffer("gbuf.pos", size, GL_R32F, }; // only depth -> reconstruct position
		col    = {"gbuf.col"    }; // rgb albedo (emmisive is simply very bright)
		norm   = {"gbuf.norm"   }; // rgb normal

		glBindTexture(GL_TEXTURE_2D, pos);
		glTexStorage2D (GL_TEXTURE_2D , 1, GL_R32F, size.x, size.y);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);

		glBindTexture(GL_TEXTURE_2D, col);
		glTexStorage2D (GL_TEXTURE_2D , 1, GL_RGB16F, size.x, size.y);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);

		glBindTexture(GL_TEXTURE_2D, norm);
		glTexStorage2D (GL_TEXTURE_2D, 1, GL_RGB16F, size.x, size.y);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);

		glBindTexture(GL_TEXTURE_2D, 0);
	}
};

// framebuffer for rendering at different resolution and to make sure we get float buffers
struct RenderPasses {
	SERIALIZE(RenderPasses, renderscale, exposure)

	Vao dummy_vao = {"dummy_vao"};

	void draw_fullscreen_triangle (StateManager& state) {
		PipelineState s;
		s.depth_test   = false;
		s.depth_write  = false;
		s.blend_enable = false;
		state.set_no_override(s);
				
		glBindVertexArray(dummy_vao);
		glDrawArrays(GL_TRIANGLES, 0, 3);
	}
	

	Renderbuffer fbo_MSAA        = {};
	Renderbuffer fbo_opaque_copy = {};
	Renderbuffer fbo             = {};
	
	render::RenderScale renderscale;
	
	Sampler fbo_sampler         = sampler("fbo_sampler", FILTER_MIPMAPPED, GL_CLAMP_TO_EDGE);
	Sampler fbo_sampler_nearest = sampler("fbo_sampler_nearest", FILTER_NEAREST, GL_CLAMP_TO_EDGE);

	Sampler fbo_sampler_bilin   = sampler("fbo_sampler_bilin", FILTER_BILINEAR, GL_CLAMP_TO_EDGE);
	
	Sampler& get_sampler () {
		return renderscale.nearest ? fbo_sampler_nearest : fbo_sampler;
	}

	static constexpr GLenum color_format = GL_R11F_G11F_B10F;// GL_RGBA16F GL_RGBA32F GL_SRGB8_ALPHA8
	//static constexpr GLenum color_format_resolve = GL_SRGB8_ALPHA8;
	static constexpr GLenum depth_format = GL_DEPTH_COMPONENT32F; // need float32 format for resonable infinite far plane (float16 only works with ~1m near plane)
	static constexpr bool color_mips = false;
	
	Shader* shad_postprocess = g_shaders.compile("postprocess");
	
	float exposure = 1.0f;
	
	void imgui () {
		renderscale.imgui();

		ImGui::SliderFloat("exposure", &exposure, 0.02f, 20.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
	}
	
	void update (int2 window_size) {
		if (renderscale.update(window_size)) {
			glActiveTexture(GL_TEXTURE0); // try clobber consistent texture at least
			
			fbo_MSAA = {};
			fbo = {};

			// create new

			//GLenum depth_format = renderscale.depth_float32 ? GL_DEPTH_COMPONENT32F : GL_DEPTH_COMPONENT16;

			if (renderscale.MSAA > 1) {
				fbo_MSAA        = Renderbuffer("fbo.MSAA",        renderscale.size, color_format, depth_format, false, renderscale.MSAA);
				fbo_opaque_copy = Renderbuffer("fbo.opaque_copy", renderscale.size, color_format, 0, false, 1);
				fbo             = Renderbuffer("fbo",             renderscale.size, color_format, 0, color_mips, 1);
			}
			else {
				fbo             = Renderbuffer("fbo",             renderscale.size, color_format, depth_format, color_mips, 1);
				fbo_opaque_copy = Renderbuffer("fbo.opaque_copy", renderscale.size, color_format, 0, false, 1);
				// leave fbo_resolve as 0
			}
		}
	}
	
	// begin primary (MSAA) fbo (opaque pass)
	void begin_primary () {
		glBindFramebuffer(GL_FRAMEBUFFER, (fbo_MSAA.fbo ? fbo_MSAA : fbo).fbo);
		
		glViewport(0, 0, renderscale.size.x, renderscale.size.y);

		glClearColor(0.01f, 0.02f, 0.03f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}
	// make copy (resolve MSAA) of primary fbo (so transparent pass can sample primary fbo for distortion)
	// still draw to primary (MSAA) after this
	void copy_primary_for_distortion () {
		OGL_TRACE("FBO.copy_primary_for_distortion");
		
		// blit MSAA fbo into normal fbo (resolve MSAA)
		glBindFramebuffer(GL_READ_FRAMEBUFFER, (fbo_MSAA.fbo ? fbo_MSAA : fbo).fbo);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo_opaque_copy.fbo);

		glBlitFramebuffer(0,0, renderscale.size.x,renderscale.size.y,
							0,0, renderscale.size.x,renderscale.size.y,
							GL_COLOR_BUFFER_BIT, GL_NEAREST); // | GL_DEPTH_BUFFER_BIT

		glBindFramebuffer(GL_FRAMEBUFFER, (fbo_MSAA.fbo ? fbo_MSAA : fbo).fbo);
	}
	// resolve final primary fbo (MSAA) (opaque + transparent) into non MSAA fbo
	// then copy and rescale into real window framebuffer (color only, linear float to srgb8, no custom shader tonemapping)
	void resolve_and_blit_to_screen (int2 window_size) {

		if (fbo_MSAA.fbo) {
			OGL_TRACE("FBO.MSAA_resolve");
			
			// blit MSAA fbo into normal fbo (resolve MSAA)
			glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_MSAA.fbo);
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo     .fbo);

			glBlitFramebuffer(0,0, renderscale.size.x,renderscale.size.y,
							  0,0, renderscale.size.x,renderscale.size.y,
							  GL_COLOR_BUFFER_BIT, GL_NEAREST);
		}
		{
			OGL_TRACE("FBO.rescale");

			// blit normal fbo into window fbo (rescale)
			glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo.fbo);
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

			glBlitFramebuffer(0,0, renderscale.size.x,renderscale.size.y,
								0,0, window_size.x,window_size.y,
								GL_COLOR_BUFFER_BIT, renderscale.nearest ? GL_NEAREST : GL_LINEAR);
		}

		// now draw to window fbo
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}
	// resolve final primary fbo (MSAA) (opaque + transparent) into non MSAA fbo
	// then draw to real window framebuffer (implement your own tonemapping shader)
	void postprocess (StateManager& state, int2 window_size) {
		if (fbo_MSAA.fbo) {
			OGL_TRACE("FBO.MSAA_resolve");
			
			// blit MSAA fbo into normal fbo (resolve MSAA)
			glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_MSAA.fbo);
			glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo     .fbo);

			glBlitFramebuffer(0,0, renderscale.size.x,renderscale.size.y,
							  0,0, renderscale.size.x,renderscale.size.y,
							  GL_COLOR_BUFFER_BIT, GL_NEAREST);
		}
		
		// now draw to window fbo
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		
		glViewport(0, 0, window_size.x, window_size.y);
		
		if (shad_postprocess->prog) {
			OGL_TRACE("postprocess");
				
			glUseProgram(shad_postprocess->prog);
				
			shad_postprocess->set_uniform("exposure", exposure);

			state.bind_textures(shad_postprocess, {
				{ "fbo_col", { GL_TEXTURE_2D, fbo.col }, get_sampler() }
			});
			draw_fullscreen_triangle(state);
		}
	}
};

} // namespace ogl
