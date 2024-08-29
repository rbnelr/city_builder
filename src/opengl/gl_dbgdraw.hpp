#pragma once
#include "opengl.hpp"
#include "agnostic_render.hpp"

namespace ogl {
	
struct glTextRenderer {
	Shader* shad;
	Texture2D atlas_tex = {"TextRenderer::atlas_tex"};
	
	VertexBufferInstancedI vbo = vertex_buffer_instancedI<TextRenderer::GlyphQuad, TextRenderer::GlyphInstance>("text");

	glTextRenderer () {
		upload_buffer(GL_ARRAY_BUFFER        , vbo.vbo, sizeof(TextRenderer::GLYPH_QUAD), TextRenderer::GLYPH_QUAD);
		upload_buffer(GL_ELEMENT_ARRAY_BUFFER, vbo.ebo, sizeof(render::shapes::QUAD_INDICES), render::shapes::QUAD_INDICES);
	}
	
	void upload_texture (Image<uint8_t>& atlas, int2 atlas_size) {
		ZoneScoped;

		glBindTexture(GL_TEXTURE_2D, atlas_tex);

		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, atlas_size.x, atlas_size.y, 0, GL_RED, GL_UNSIGNED_BYTE, atlas.pixels);

		glGenerateMipmap(GL_TEXTURE_2D);

		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
		//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, mipmaps);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

		glBindTexture(GL_TEXTURE_2D, 0);
	}

	void begin (TextRenderer& text) {
		text.begin();
		if (text.texture_changed) {
			shad = g_shaders.compile("text", {{"SDF", text.enable_sdf ? "1":"0"}});

			auto atlas_img = text.generate_tex();
			upload_texture(atlas_img, text.atlas_size);

			text.texture_changed = false;
		}
	}

	void render (StateManager& state, TextRenderer& text) {
		OGL_TRACE("TextRenderer")
		ZoneScoped;

		glUseProgram(shad->prog);

		PipelineState s;
		s.depth_test = false;
		s.blend_enable = true; 
		s.blend_func.equation = GL_FUNC_ADD; 
		s.blend_func.sfactor = GL_ONE; // for outlined SDF math it's simpler to multiply alpha ourselves
		s.blend_func.dfactor = GL_ONE_MINUS_SRC_ALPHA;
		state.set(s);

		state.bind_textures(shad, {
			{ "atlas_tex", atlas_tex }
		});

		if (text.enable_sdf) {
			shad->set_uniform("sdf_onedge", (float)text.sdf_onedge / 255.0f);
			shad->set_uniform("sdf_scale", 255.0f / text.sdf_scale);

			shad->set_uniform("sdf_outline_w", text.sdf_outline ? text.sdf_outline_w : 0.0f);
			shad->set_uniform("sdf_outline_col", text.sdf_outline_col);
			shad->set_uniform("sdf_grow", text.sdf_grow);
		}

		vbo.stream_instances(text.glyph_instances);

		glBindVertexArray(vbo.vao);
		glDrawElementsInstanced(GL_TRIANGLES, (GLsizei)ARRLEN(render::shapes::QUAD_INDICES),
			GL_UNSIGNED_SHORT, (void*)0, (GLsizei)text.glyph_instances.size());
	}
};

struct glDebugDraw {
	bool wireframe          = false;
	bool wireframe_no_cull  = false;
	bool wireframe_no_blend = true;

	float line_width = 2;
	
	Shader* shad_lines = g_shaders.compile("dbg_lines");
	Shader* shad_tris  = g_shaders.compile("dbg_tris");

	VertexBuffer vbo_lines = vertex_buffer<render::DebugDraw::LineVertex>("Dbg.LineVertex");
	VertexBuffer vbo_tris  = vertex_buffer<render::DebugDraw::TriVertex>("Dbg.TriVertex");

	glTextRenderer gl_text_render;

	struct IndirectLineVertex {
		float4 pos; // vec4 for std430 alignment
		float4 col;
	
		VERTEX_CONFIG(
			ATTRIB(FLT,4, IndirectLineVertex, pos),
			ATTRIB(FLT,4, IndirectLineVertex, col),
		)
	};
	
	static constexpr int IndirectBufferMaxLines = 1024*64;

	struct IndirectBufferCmd {
		int update;
		int max_lines;
		glDrawArraysIndirectCommand cmd;
	};
	struct IndirectBuffer {
		IndirectBufferCmd cmd_and_more;
		int _pad[2];
		IndirectLineVertex lines[IndirectBufferMaxLines];
	};
	
	// indirect_vbo contains all indirect drawing data
	// Using Vbo without setup_vao? use Ssbo instead?
	Vbo indirect_vbo = {"DebugDraw.indirect_draw"}; // = 1x IndirectBuffer
	
	Vao indirect_lines_vao = {"DebugDraw.indirect_lines"};
	
	bool update_indirect = false;
	bool accum_indirect = false;
	bool _clear_indirect = false;

	void clear_indirect () {
		glBindBuffer(GL_ARRAY_BUFFER, indirect_vbo);
	
		{
			IndirectBufferCmd cmd = {};
			cmd.update = (int)update_indirect;
			cmd.max_lines = IndirectBufferMaxLines;
			cmd.cmd.instanceCount = 1;
			glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(cmd), &cmd);
		}

		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	void update (Input& I) {
		if (I.buttons[KEY_T].went_down) {
			if (I.buttons[KEY_LEFT_SHIFT].is_down) {
				update_indirect = false;
				_clear_indirect = true;
			} else {
				update_indirect = !update_indirect;
			}
		}

		glBindBuffer(GL_ARRAY_BUFFER, indirect_vbo);
		int intbool = (int)update_indirect;
		glBufferSubData(GL_ARRAY_BUFFER, offsetof(IndirectBuffer, cmd_and_more.update), sizeof(int), &intbool);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		if (_clear_indirect || (update_indirect && !accum_indirect)) {
			clear_indirect();
			_clear_indirect = false;
		}
	}

	glDebugDraw () {
		setup_vao(IndirectLineVertex::attribs(), indirect_lines_vao, indirect_vbo, 0, offsetof(IndirectBuffer, lines[0]));
		
		glBindBuffer(GL_ARRAY_BUFFER, indirect_vbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(IndirectBuffer), nullptr, GL_STREAM_DRAW);
		clear_indirect();

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	void imgui (TextRenderer& text) {
		if (ImGui::TreeNode("Debug Draw")) {

			ImGui::Checkbox("wireframe", &wireframe);
			ImGui::SameLine();
			ImGui::Checkbox("no cull", &wireframe_no_cull);
			ImGui::SameLine();
			ImGui::Checkbox("no blend", &wireframe_no_blend);

			ImGui::SliderFloat("line_width", &line_width, 1.0f, 8.0f);
			
			ImGui::Checkbox("update_indirect [T]", &update_indirect);
			_clear_indirect = ImGui::Button("clear_indirect") || _clear_indirect;
			ImGui::SameLine();
			ImGui::Checkbox("accum_indirect", &accum_indirect);

			text.imgui();

			ImGui::TreePop();
		}
	}

	void render (StateManager& state, render::DebugDraw& dbg) {
		OGL_TRACE("debug_draw");

		ZoneScoped;
		
		if (shad_lines->prog) {
			OGL_TRACE("Dbg.Lines");

			glUseProgram(shad_lines->prog);

			PipelineState s;
			s.depth_test = false;
			s.blend_enable = true;
			state.set_no_override(s);

			vbo_lines.stream(dbg.lines);

			if (dbg.lines.size() > 0) {
				glBindVertexArray(vbo_lines.vao);
				glDrawArrays(GL_LINES, 0, (GLsizei)dbg.lines.size());
			}

			{
				glMemoryBarrier(GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT|GL_COMMAND_BARRIER_BIT);
				
				glBindVertexArray(indirect_lines_vao);
				glBindBuffer(GL_DRAW_INDIRECT_BUFFER, indirect_vbo);
				
				size_t cmd_offs = offsetof(IndirectBuffer, cmd_and_more.cmd);
				glDrawArraysIndirect(GL_LINES, (void*)cmd_offs);

				glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
			}
		}

		if (shad_tris->prog) {
			OGL_TRACE("Dbg.Tris");
		
			vbo_tris.stream(dbg.tris);

			if (dbg.tris.size() > 0) {
				glUseProgram(shad_tris->prog);

				PipelineState s;
				s.depth_test = false;
				s.blend_enable = true;
				state.set(s);

				glBindVertexArray(vbo_tris.vao);
				glDrawArrays(GL_TRIANGLES, 0, (GLsizei)dbg.tris.size());
			}
		}
		
		glBindVertexArray(0);

		
		gl_text_render.render(state, dbg.text);
	}
};

}
