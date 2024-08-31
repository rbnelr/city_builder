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
	SERIALIZE(BloomRenderer, bloom_fac, filter_radius, low_mip, prefilter_clamp, prefilter_clamp_val, prefilter_threshold, prefilter_knee)
	
	Shader* shad_downsample0 = g_shaders.compile("post_bloom", {{"PASS","0"}});
	Shader* shad_downsample  = g_shaders.compile("post_bloom", {{"PASS","1"}});
	Shader* shad_upsample    = g_shaders.compile("post_bloom", {{"PASS","2"}});

	Render_Texture downsample;
	Render_Texture upsample;

	Sampler sampler = make_sampler("bloom sampler", FILTER_MIPMAPPED, GL_CLAMP_TO_EDGE);

	bool _enable = true; // also deallocate entire class if disabled like in shadow renderer
	float bloom_fac = 0.05f;

	float filter_radius = 1.50f;

	int2 full_res;
	int max_mips = 0;
	int mips = 0;

	int low_mip = 4;

	bool prefilter_clamp = false;
	float prefilter_clamp_val = 20;
	float prefilter_threshold = 1;
	float prefilter_knee = 0.125f;

	bool need_resize = true;
	
	std::vector<int2> mip_sizes;
	std::vector<Fbo> down_fbos;
	std::vector<Fbo> up_fbos;

	int _visualize_mip = 0;

	void imgui () {
		if (imgui_Header("Bloom")) {
			ImGui::Checkbox("Enable", &_enable);
			ImGui::SliderFloat("Bloom Strength", &bloom_fac, 0,1);
			
			ImGui::SliderFloat("Filter Radius", &filter_radius, 0,5);

			ImGui::SetNextItemWidth(-160);
			if (ImGui::SliderInt("Lowest mip", &low_mip, 0, max_mips-2))
				need_resize = true;
			ImGui::SameLine();
			int2 low_res = max(full_res >> (mips-1), 1);
			ImGui::Text("#%d - %dx%d", mips-1, low_res.x, low_res.y);

			ImGui::SeparatorText("Prefilter");
			ImGui::SetNextItemWidth(-200);
			ImGui::Checkbox("Clamp", &prefilter_clamp);
			ImGui::SameLine();
			ImGui::SetNextItemWidth(200);
			ImGui::DragFloat("##clamp_val", &prefilter_clamp_val, 0.1f);

			ImGui::SliderFloat("Threshold", &prefilter_threshold, 0,3);
			ImGui::SliderFloat("Knee", &prefilter_knee, 0,prefilter_threshold);
			prefilter_knee = min(prefilter_knee, prefilter_threshold);

			ImGui::SliderInt("_visualize_mip", &_visualize_mip, 0, mips-1);

			ImGui::PopID();
		}
	}

	void resize (int2 lighting_res) {
		full_res = max(lighting_res / 2, 1);
		max_mips = calc_mipmaps(full_res.x, full_res.y);
		mips = clamp(max_mips - low_mip, 1, max_mips);

		downsample = Render_Texture("bloom_downsample", full_res, post_light_col_format, mips);
		upsample   = Render_Texture("bloom_upsample"  , full_res, post_light_col_format, mips-1);

		mip_sizes.resize(mips);
		down_fbos.resize(mips);
		up_fbos.resize(mips-1);

		for (int mip=0; mip < mips  ; mip++) down_fbos[mip] = single_attach_fbo("bloom_down_fbo", downsample, mip);
		for (int mip=0; mip < mips-1; mip++) up_fbos  [mip] = single_attach_fbo("bloom_up_fbo", upsample, mip);
		
		int2 res = full_res;
		for (int mip=0; mip<mips; mip++) {
			mip_sizes[mip] = res;
			res = max(res / 2, 1);
		}

		need_resize = false;
	}

	void render (StateManager& state, LightingFbo& light_buf) {
		ZoneScoped;
		OGL_TRACE("bloom");
		
		glUseProgram(shad_downsample0->prog);

		{
			auto& res = mip_sizes[0];

			glBindFramebuffer(GL_FRAMEBUFFER, down_fbos[0]);
			glViewport(0, 0, res.x, res.y);
		
			state.bind_textures(shad_downsample0, {
				{ "input", light_buf.col, sampler }
			});
			shad_downsample0->set_uniform("input_mip", 0.0f);
			shad_downsample0->set_uniform("texel_sz", 1.0f / (float2)res);
			
			assert(prefilter_knee <= prefilter_threshold);
			shad_downsample0->set_uniform("prefilter_clamp", prefilter_clamp ? prefilter_clamp_val : INF);
			shad_downsample0->set_uniform("prefilter_threshold", prefilter_threshold);
			shad_downsample0->set_uniform("prefilter_knee", max(prefilter_knee, 0.001f));

			draw_fullscreen_triangle(state);
		}

		glUseProgram(shad_downsample->prog);

		for (int mip=1; mip<mips; mip++) {
			auto& res = mip_sizes[mip];

			glBindFramebuffer(GL_FRAMEBUFFER, down_fbos[mip]);
			glViewport(0, 0, res.x, res.y);
		
			state.bind_textures(shad_downsample, {
				{ "input", downsample, sampler }
			});
			shad_downsample->set_uniform("input_mip", (float)(mip-1));
			shad_downsample->set_uniform("texel_sz", 1.0f / (float2)res);

			draw_fullscreen_triangle(state);
		}

		glUseProgram(shad_upsample->prog);

		for (int mip=mips-2; mip>=0; mip--) {
			auto& res = mip_sizes[mip];

			glBindFramebuffer(GL_FRAMEBUFFER, up_fbos[mip]);
			glViewport(0, 0, res.x, res.y);

			state.bind_textures(shad_upsample, {
				{ "blur_tex", mip == mips-2 ? downsample : upsample, sampler },
				{ "add_tex",  downsample, sampler },
			});
			shad_upsample->set_uniform("blur_mip", (float)(mip+1));
			shad_upsample->set_uniform("add_mip", (float)mip);
			shad_upsample->set_uniform("texel_sz", 1.0f / (float2)res);
			shad_upsample->set_uniform("filter_radius", filter_radius);

			draw_fullscreen_triangle(state);
		}

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}
};

} // namespace ogl
