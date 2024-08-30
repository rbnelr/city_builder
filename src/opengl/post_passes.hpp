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
	Shader* shad_upsample   = g_shaders.compile("post_bloom", {{"PASS","2"}});

	Render_Texture downsample;
	Render_Texture upsample;

	Sampler sampler = make_sampler("bloom sampler", FILTER_MIPMAPPED, GL_CLAMP_TO_EDGE);

	bool _enable = false; // also deallocate entire class if disabled like in shadow renderer
	
	int2 full_res;
	int max_mips = 0;
	int mips = 0;

	int low_mip = 4;

	std::vector<Fbo> down_fbos;
	std::vector<Fbo> up_fbos;

	int _visualize_mip = 0;

	void imgui () {
		if (ImGui::TreeNode("Bloom")) {
			ImGui::Checkbox("enable", &_enable);

			ImGui::SliderInt("Lowest mip", &low_mip, 0, max_mips-2);
			ImGui::SameLine();
			int2 low_res = full_res >> (mips-1);
			ImGui::Text("#%d - %dx%d", mips-1, low_res.x, low_res.y);
#
			ImGui::SliderInt("_visualize_mip", &_visualize_mip, 0, mips-1);

			ImGui::TreePop();
		}
	}

	void resize (int2 lighting_res) {
		full_res = max(lighting_res / 2, 1);
		max_mips = calc_mipmaps(full_res.x, full_res.y);
		mips = clamp(max_mips - low_mip, 0, max_mips);

		downsample = Render_Texture("bloom_downsample", full_res, post_light_col_format, mips);
		upsample   = Render_Texture("bloom_upsample"  , full_res, post_light_col_format, mips-1);

		down_fbos.resize(mips);
		up_fbos.resize(mips-1);
		for (int mip=0; mip < mips  ; mip++) down_fbos[mip] = single_attach_fbo("bloom_up_fbo", downsample, mip);
		for (int mip=0; mip < mips-1; mip++) up_fbos  [mip] = single_attach_fbo("bloom_down_fbo", downsample, mip);
	}

	void render (StateManager& state, LightingFbo& light_buf) {
		ZoneScoped;
		OGL_TRACE("bloom");
		
		glUseProgram(shad_downsample->prog);

		int2 res = full_res;
		for (int mip=0; mip <= mips; mip++) {
			glBindFramebuffer(GL_FRAMEBUFFER, down_fbos[mip]);
			glViewport(0, 0, res.x, res.y);
		
			state.bind_textures(shad_downsample, {
				{ "input", mip == 0 ? light_buf.col : downsample, sampler }
			});
			shad_downsample->set_uniform("input_mip", (float)(mip == 0 ? 0 : mip-1));
			shad_downsample->set_uniform("texel_sz", 1.0f / (float2)res);

			draw_fullscreen_triangle(state);

			res = max(res / 2, 1);
		}

		glUseProgram(shad_upsample->prog);

		int2 res = full_res;
		for (int mip = last_mip-1; mip >= 0; mip--) {
			glBindFramebuffer(GL_FRAMEBUFFER, up_fbos[mip]);
			glViewport(0, 0, res.x, res.y);

			bool first = mip == mips-1 - low_mip;
			state.bind_textures(shad_downsample, {
				{ "blur_tex", first ? downsample : upsample },
				{ "add_tex",  downsample },
			});
			shad_downsample->set_uniform("blur_mip", (float)(mip == 0 ? 0 : mip-1));
			shad_downsample->set_uniform("add_mip", (float)(mip == 0 ? 0 : mip-1));
			shad_downsample->set_uniform("texel_sz", 1.0f / (float2)res);

			draw_fullscreen_triangle(state);

			res = max(res / 2, 1);
		}

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}
};

} // namespace ogl
