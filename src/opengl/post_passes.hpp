#pragma once
#include "common.hpp"
#include "ogl_render.hpp"

namespace ogl {
	
class LightingFbo {
	MOVE_ONLY_CLASS(LightingFbo); // No move implemented for now
public:
	// Need to include gbuf depth buffer in lighting fbo because point light require depth testing (but not depth writing)
	// at the same time we can't apply defferred point lights to the gbuf itself, since albedo need to stay around, so we need this seperate render target
	// attaching the existing depth to a second FBO should be the correct solution
	
	static constexpr GLenum format = GL_RGB16F; // R11F_G11F_B10F
	
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

		col = Render_Texture(lbl+".col", size, format, levels);

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
	SERIALIZE(BloomRenderer, bloom_fac, filter_radius, prefilter_clamp, prefilter_clamp_val, prefilter_threshold, prefilter_knee)
	
	static constexpr GLenum format = GL_R11F_G11F_B10F;

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

	static constexpr int low_mip = 2;

	bool prefilter_clamp = false;
	float prefilter_clamp_val = 20;
	float prefilter_threshold = 1;
	float prefilter_knee = 0.125f;

	bool need_resize = true;
	
	std::vector<Fbo> down_fbos;
	std::vector<Fbo> up_fbos;

	int _visualize_mip = 0;

	void imgui () {
		if (imgui_Header("Bloom")) {
			ImGui::Checkbox("Enable", &_enable);
			ImGui::SliderFloat("Bloom Strength", &bloom_fac, 0,1);
			
			ImGui::SliderFloat("Filter Radius", &filter_radius, 0,5);

			//ImGui::SetNextItemWidth(-160);
			//if (ImGui::SliderInt("Lowest mip", &low_mip, 0, max_mips-2))
			//	need_resize = true;
			//ImGui::SameLine();
			int2 low_res = calc_mip_res(full_res, mips-1);
			//ImGui::Text("#%d - %dx%d", mips-1, low_res.x, low_res.y);

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
		assert(lighting_res.x > 8 && lighting_res.y > 8); // code kinda breaks with 1x1 render res!

		full_res = max(lighting_res / 2, 1);
		max_mips = calc_mipmaps(full_res);
		mips = clamp(max_mips - low_mip, 1, max_mips);

		downsample = Render_Texture("bloom_downsample", full_res, format, mips);
		upsample   = Render_Texture("bloom_upsample"  , full_res, format, mips-1);

		down_fbos.resize(mips);
		up_fbos.resize(mips-1);

		for (int mip=0; mip < mips  ; mip++) down_fbos[mip] = single_attach_fbo("bloom_down_fbo", downsample, mip);
		for (int mip=0; mip < mips-1; mip++) up_fbos  [mip] = single_attach_fbo("bloom_up_fbo", upsample, mip);
		
		need_resize = false;
	}

	void invalidate () {
		for (int mip=0; mip<mips; mip++)
			glInvalidateTexImage(downsample, mip);

		for (int mip=0; mip<mips-1; mip++)
			glInvalidateTexImage(upsample, mip);
	}

	void render (StateManager& state, LightingFbo& light_buf) {
		ZoneScoped;
		OGL_TRACE("bloom");
		
		glUseProgram(shad_downsample0->prog);

		{
			auto res = calc_mip_res(full_res, 0);

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
			auto res = calc_mip_res(full_res, mip);

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
			auto res = calc_mip_res(full_res, mip);

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

struct ExposureReadback {
	int buffers = 2;
	int counter = 0;
	std::vector<DownloadPbo> pbos;

	ExposureReadback () {
		pbos.resize(buffers);
		for (int i=0; i<buffers; ++i) {
			pbos[i] = DownloadPbo{prints("ExposureReadback[%d]", i)};
		}
		counter = 0;
	}

	int desired_res = 4;
	float2 edge_weight = 0.0f;

	bool readback (Render_Texture& tex, int2 full_res, lrgb* weighted_average) {
		ZoneScoped;
		OGL_TRACE("exposure readback");
		
		int cur_buf = counter;
		counter = wrap(counter+1, buffers);
		int oldest_buf = counter;

		int mips = calc_mipmaps(full_res);
		int mip = clamp(mips - desired_res, 0, mips-1);
		auto res = calc_mip_res(full_res, mip);

		int size = res.x * res.y * sizeof(lrgb);
	
		{ // Trigger read into current pbo
			ZoneScopedN("glGetTexImage");
			OGL_TRACE("glGetTexImage");
			
			glBindBuffer(GL_PIXEL_PACK_BUFFER, pbos[cur_buf]);
			glBufferData(GL_PIXEL_PACK_BUFFER, size, nullptr, GL_STREAM_DRAW); // TODO: avoid reallocating?
		
			glBindTexture(GL_TEXTURE_2D, tex);
			glGetTexImage(GL_TEXTURE_2D, mip, GL_RGB, GL_FLOAT, nullptr);
			glBindTexture(GL_TEXTURE_2D, 0);
		}
		
		bool readback_avail = false;
		{ // Map now available oldest pbo
			ZoneScopedN("glMapBuffer");
			OGL_TRACE("glMapBuffer");

			glBindBuffer(GL_PIXEL_PACK_BUFFER, pbos[oldest_buf]);

			auto* mapped = (lrgb*)glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
			if (mapped) {
				lrgb total = 0;
				float total_weight = 0;

				for (int y=0; y<res.y; ++y)
				for (int x=0; x<res.x; ++x) {
					float2 uv = ((float2)int2(x,y) + 0.5f) / (float2)res;
					float2 t2d = lerp(edge_weight, 1.0f, 1 - abs(uv * 2 - 1));
					float t = min(t2d.x, t2d.y);

					total += mapped[x + y*res.x];
					total_weight += t;
				}

				*weighted_average = total / total_weight;
				readback_avail = true;
		
				glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
				glInvalidateBufferData(pbos[oldest_buf]);
			}
		}
		
		glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

		return readback_avail;
	}
};

struct ExposureControl {
	SERIALIZE(ExposureControl, auto_exposure, exposure, adjust_speed, brightness)

	bool auto_exposure = true;
	float exposure = 0.0f;
	float adjust_speed = 5.0f;
	float brightness = 0.3f;
	
	float exposure_mult () {
		return powf(2.0f, exposure);
	}

	float last_luma = 1.0f;

	ExposureReadback backreader;
	
	void imgui () {
		if (!imgui_Header("Exposure", true)) return;

		ImGui::Checkbox("Auto Exposure", &auto_exposure);
		ImGui::SliderFloat("Exposure", &exposure, -10, +20);
		
		ImGui::SliderFloat("Adjust Speed", &adjust_speed, 0, 10);
		ImGui::SliderFloat("Brightness", &brightness, 0.05f, 2);
		
		ImGui::Text("Exposure for %10.5f  Cur Avg Brightness: %.2f", exposure_mult(), last_luma);

		ImGui::PopID();
	}

	void auto_exposure_readback (Render_Texture& tex, int2 full_res, float dt) {
		ZoneScoped;

		return;

		lrgb avg_exposed_rgb;
		if (backreader.readback(tex, full_res, &avg_exposed_rgb)) {
			float luma = lrgb_luminance(avg_exposed_rgb);

			if (auto_exposure) {
				float delta = log2f(max(brightness, 0.001f)) - log2f(luma);
				float dir = delta > 0 ? +1.0f : -1.0f;
				delta = abs(delta);
				float speed = dt * adjust_speed * clamp(delta*delta, 0.1f, 2.0f);

				exposure += dir * min(speed, max(delta - 0.25f, 0.0f));
			}

			last_luma = luma;
		}
	}
};


} // namespace ogl
