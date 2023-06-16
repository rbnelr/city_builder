#pragma once
#include "common.hpp"
#include "app.hpp"
#include "engine/opengl.hpp"
#include "gl_dbgdraw.hpp"

namespace ogl {

struct TriRenderer {
	Shader* shad  = g_shaders.compile("tris");

	Texture2D tex = texture2D<srgba8>("logo", "textures/Opengl-logo.png");

	Sampler sampler_normal = sampler("sampler_normal", FILTER_MIPMAPPED, GL_REPEAT, false);

	struct Vertex {
		float2 pos;
		float2 uv;
		float4 col;

		VERTEX_CONFIG(
			ATTRIB(FLT2, Vertex, pos),
			ATTRIB(FLT2, Vertex, uv),
			ATTRIB(FLT4, Vertex, col),
		)
	};

	VertexBufferI vbo_tris = vertex_bufferI<Vertex>("TriRenderer.Vertex");

	std::vector<Vertex>   verticies;
	std::vector<uint16_t> indices;

	void update (Input& I) {
		verticies.clear();
		verticies.shrink_to_fit();
		indices.clear();
		indices.shrink_to_fit();
	}

	void push_quad (float2 pos, float2 size, float4 col) {
		uint16_t idx = (uint16_t)verticies.size();

		auto* pv = push_back(verticies, 4);
		pv[0] = { pos + float2(     0,      0), float2(0,0), col };
		pv[1] = { pos + float2(size.x,      0), float2(1,0), col };
		pv[2] = { pos + float2(size.x, size.y), float2(1,1), col };
		pv[3] = { pos + float2(     0, size.y), float2(0,1), col };

		render::shapes::push_quad_indices<uint16_t>(indices, idx+0u, idx+1u, idx+2u, idx+3u);
	}

	void render (StateManager& state) {
		OGL_TRACE("TriRenderer");

		ZoneScoped;

		if (shad->prog) {
			OGL_TRACE("TriRenderer");

			vbo_tris.stream(verticies, indices);

			if (indices.size() > 0) {
				glUseProgram(shad->prog);

				state.bind_textures(shad, {
					{ "tex", tex, sampler_normal }
				});

				PipelineState s;
				s.depth_test = false;
				s.blend_enable = true;
				state.set(s);

				glBindVertexArray(vbo_tris.vao);
				glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_UNSIGNED_SHORT, (void*)0);
			}
		}

		glBindVertexArray(0);
	}
};

struct OglRenderer : public Renderer {
	SERIALIZE_NONE(OglRenderer)
	
	virtual void imgui (App& app) {
		if (imgui_Header("Renderer", true)) {
			
		#if OGL_USE_REVERSE_DEPTH
			ImGui::Checkbox("reverse_depth", &ogl::reverse_depth);
		#endif

			gl_dbgdraw.imgui();

			ImGui::PopID();
		}
	}

	StateManager state;
	
	struct CommonUniforms {
		static constexpr int UBO_BINDING = 0;

		Ubo ubo = {"common_ubo"};

		struct Common {
			View3D view;
		};

		void set (View3D const& view) {
			Common common = {};
			common.view = view;
			stream_buffer(GL_UNIFORM_BUFFER, ubo, sizeof(common), &common, GL_STREAM_DRAW);

			glBindBuffer(GL_UNIFORM_BUFFER, ubo);
			glBindBufferBase(GL_UNIFORM_BUFFER, UBO_BINDING, ubo);
			glBindBuffer(GL_UNIFORM_BUFFER, 0);
		}
	};
	CommonUniforms common_ubo;

	glDebugDraw gl_dbgdraw;
	
	TriRenderer tri_renderer;

	OglRenderer () {
		
	}
	
	virtual void begin (App& app) {
		
	}
	virtual void end (App& app) {
		ZoneScoped;
		
		{
			OGL_TRACE("setup");

			{
				//OGL_TRACE("set state defaults");

				state.wireframe          = gl_dbgdraw.wireframe;
				state.wireframe_no_cull  = gl_dbgdraw.wireframe_no_cull;
				state.wireframe_no_blend = gl_dbgdraw.wireframe_no_blend;

				state.set_default();

				glEnable(GL_LINE_SMOOTH);
				//glLineWidth(debug_draw.line_width);
				glLineWidth(2);
			}

			{
				common_ubo.set(app.view);

				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, gl_dbgdraw.indirect_vbo);

				gl_dbgdraw.update(app.input);
			}
		}
		
		{
			glViewport(0,0, app.input.window_size.x, app.input.window_size.y);
			glClearColor(0.01f, 0.012f, 0.014f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT);
		}

		gl_dbgdraw.render(state, dbgdraw);

		tri_renderer.update(app.input);
		tri_renderer.push_quad(float2(0,0), 1, float4(1,0,0,1));
		tri_renderer.push_quad(float2(2,0), 1, float4(0,1,0,1));
		tri_renderer.push_quad(float2(0,2), 1.5f, float4(0,0,1,1));
		tri_renderer.render(state);

		{
			OGL_TRACE("draw ui");
		
			if (app.trigger_screenshot && !app.screenshot_hud) take_screenshot(app.input.window_size);
		
			// draw HUD

			app.draw_imgui();

			if (app.trigger_screenshot && app.screenshot_hud)  take_screenshot(app.input.window_size);
			app.trigger_screenshot = false;
		}
	}
};

} // namespace ogl

std::unique_ptr<Renderer> create_ogl_backend () {
	return std::make_unique<ogl::OglRenderer>();
}
