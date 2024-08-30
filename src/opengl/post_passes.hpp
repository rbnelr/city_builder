#pragma once
#include "common.hpp"
#include "ogl_common.hpp"

namespace ogl {
	
constexpr GLenum post_light_col_format = GL_RGB16F; // R11F_G11F_B10F

class LightingFbo {
	MOVE_ONLY_CLASS(LightingFbo); // No move implemented for now
public:
	// Need to include gbuf depth buffer in lighting fbo because point light require depth testing (but not depth writing)
	// at the same time we can't apply defferred point lights to the gbuf itself, since albedo need to stay around, so we need this seperate render target
	// attaching the existing depth to a second FBO should be the correct solution

	static void swap (LightingFbo& l, LightingFbo& r) {
		std::swap(l.fbo, r.fbo);
		std::swap(l.col, r.col);
	}

	Fbo fbo = {};
	Render_Texture col = {};

	LightingFbo () {}
	LightingFbo (std::string_view label, int2 size, Render_Texture& depth, bool mips=false) {
		GLint levels = mips ? calc_mipmaps(size.x, size.y) : 1;

		std::string lbl = (std::string)label;

		col = Render_Texture(lbl+".col", size, post_light_col_format, levels);

		{
			fbo = Fbo(lbl+".fbo");
			glBindFramebuffer(GL_FRAMEBUFFER, fbo);

			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, col, 0);
			glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth, 0);

			GLuint bufs[] = { GL_COLOR_ATTACHMENT0 };
			glDrawBuffers(ARRLEN(bufs), bufs);
		
			GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
			if (status != GL_FRAMEBUFFER_COMPLETE) {
				fatal_error("glCheckFramebufferStatus: %x\n", status);
			}
		}

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	void invalidate () {
		glInvalidateTexImage(col, 0);
	}
};

struct BloomRenderer {
	
	Shader* shad_downsample = g_shaders.compile("post_bloom", {{"PASS","1"}});

	Render_Texture downsample;
	Render_Texture upsample;

	Sampler sampler = make_sampler("bloom sampler", FILTER_BILINEAR, GL_CLAMP_TO_EDGE);

	int2 res;
	int mips;

	std::vector<Fbo> down_fbos;

	void resize (int2 lighting_res) {
		res = max(lighting_res / 2, 1);
		mips = calc_mipmaps(res.x, res.y);

		downsample = Render_Texture("bloom_downsample", res, post_light_col_format, mips);
		upsample   = Render_Texture("bloom_upsample"  , res, post_light_col_format, mips);

		down_fbos.resize(mips);
		for (int mip=0; mip<mips; mip++) {
			down_fbos[mip] = single_attach_fbo("bloom_up_fbo", downsample, mip);
		}
	}

	void render (StateManager& state, LightingFbo& light_buf) {
		ZoneScoped;
		OGL_TRACE("bloom");
		
		glUseProgram(shad_downsample->prog);
		
		{
			glBindFramebuffer(GL_FRAMEBUFFER, down_fbos[0]);
			glViewport(0, 0, res.x, res.y);
		
			state.bind_textures(shad_downsample, {
				{ "input", light_buf.col, sampler }
			});
			draw_fullscreen_triangle(state);
		}

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}
};

} // namespace ogl
