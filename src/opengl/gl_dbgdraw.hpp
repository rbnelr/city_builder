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


	//VertexBufferInstancedI vbo_wire_cube   = indexed_instanced_buffer<DebugDraw::PosVertex, DebugDraw::Instance>("DebugDraw.vbo_wire_cube");
	//VertexBufferInstancedI vbo_wire_sphere = indexed_instanced_buffer<DebugDraw::PosVertex, DebugDraw::Instance>("DebugDraw.vbo_wire_sphere");
	
	struct IndirectLineVertex {
		float4 pos; // vec4 for std430 alignment
		float4 col;
	
		VERTEX_CONFIG(
			ATTRIB(FLT,4, IndirectLineVertex, pos),
			ATTRIB(FLT,4, IndirectLineVertex, col),
		)
	};
	//struct IndirectWireInstace {
	//	float4 pos; // vec4 for std430 alignment
	//	float4 size; // vec4 for std430 alignment
	//	float4 col;
	//
	//	template <typename ATTRIBS>
	//	static void attributes (ATTRIBS& a) {
	//		a.init(sizeof(IndirectWireInstace), true);
	//		a.template add<AttribMode::FLOAT, decltype(pos)>(1, "instance_pos", offsetof(IndirectWireInstace, pos));
	//		a.template add<AttribMode::FLOAT, decltype(size)>(2, "instance_size", offsetof(IndirectWireInstace, size));
	//		a.template add<AttribMode::FLOAT, decltype(col)>(3, "instance_col", offsetof(IndirectWireInstace, col));
	//	}
	//};
	
	struct IndirectBuffer {
		bool update;
		char _pad[15];

		struct Lines {
			glDrawArraysIndirectCommand cmd;
			IndirectLineVertex vertices[4096*2];
		} lines;
	
		//struct WireInstances {
		//	glDrawElementsIndirectCommand cmd;
		//	uint32_t _pad[3]; // padding for std430 alignment
		//	IndirectWireInstace vertices[4096*2];
		//};
		//
		//WireInstances wire_cubes;
		//WireInstances wire_spheres;
	};
	
	// indirect_vbo contains all indirect drawing data
	// Using Vbo without setup_vao? use Ssbo instead?
	Vbo indirect_vbo = {"DebugDraw.indirect_draw"}; // = 1x IndirectBuffer
	
	Vao indirect_lines_vao = {"DebugDraw.indirect_lines"};
	
	//Vao indirect_wire_cube_vao = setup_instanced_vao<DebugDraw::PosVertex, IndirectWireInstace>("DebugDraw.indirect_wire_cubes", 
	//	{ indirect_vbo, offsetof(IndirectBuffer, wire_cubes.vertices[0]) },
	//	vbo_wire_cube.mesh_vbo, vbo_wire_cube.mesh_ebo);
	//
	//Vao indirect_wire_sphere_vao = setup_instanced_vao<DebugDraw::PosVertex, IndirectWireInstace>("DebugDraw.indirect_wire_spheres", 
	//	{ indirect_vbo, offsetof(IndirectBuffer, wire_spheres.vertices[0]) },
	//	vbo_wire_sphere.mesh_vbo, vbo_wire_sphere.mesh_ebo);
	//
	//int wire_sphere_indices_count;
	
	bool update_indirect = false;
	bool accum_indirect = false;
	bool _clear_indirect = false;

	void clear_indirect () {
		glBindBuffer(GL_ARRAY_BUFFER, indirect_vbo);
	
		{
			glDrawArraysIndirectCommand cmd = {};
			cmd.instanceCount = 1;
			glBufferSubData(GL_ARRAY_BUFFER, offsetof(IndirectBuffer, lines.cmd), sizeof(cmd), &cmd);
		}
	
		//{
		//	glDrawElementsIndirectCommand cmd = {};
		//	cmd.count = ARRLEN(DebugDraw::_wire_cube_indices);
		//	glBufferSubData(GL_ARRAY_BUFFER, offsetof(IndirectBuffer, wire_cubes.cmd), sizeof(cmd), &cmd);
		//}
		//
		//{
		//	glDrawElementsIndirectCommand cmd = {};
		//	cmd.count = wire_sphere_indices_count;
		//	glBufferSubData(GL_ARRAY_BUFFER, offsetof(IndirectBuffer, wire_spheres.cmd), sizeof(cmd), &cmd);
		//}

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
		glBufferSubData(GL_ARRAY_BUFFER, offsetof(IndirectBuffer, update), sizeof(bool), &update_indirect);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		if (_clear_indirect || (update_indirect && !accum_indirect)) {
			clear_indirect();
			_clear_indirect = false;
		}
	}

	glDebugDraw () {
		setup_vao(IndirectLineVertex::attribs(), indirect_lines_vao, indirect_vbo, 0, offsetof(IndirectBuffer, lines.vertices[0]));
		
		//{
		//	glBindBuffer(GL_ARRAY_BUFFER, vbo_wire_cube.mesh_vbo);
		//	glBufferData(GL_ARRAY_BUFFER, sizeof(DebugDraw::_wire_cube_vertices), DebugDraw::_wire_cube_vertices, GL_STATIC_DRAW);
		//
		//	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_wire_cube.mesh_ebo);
		//	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(DebugDraw::_wire_cube_indices), DebugDraw::_wire_cube_indices, GL_STATIC_DRAW);
		//}
		//
		//{
		//	std::vector<uint16_t> indices;
		//	std::vector<float3> vertices;
		//	DebugDraw::gen_simple_wire_sphere(&vertices, &indices, 0.5f, 24);
		//
		//	wire_sphere_indices_count = (int)indices.size();
		//
		//	glBindBuffer(GL_ARRAY_BUFFER, vbo_wire_sphere.mesh_vbo);
		//	glBufferData(GL_ARRAY_BUFFER, sizeof(float3)*vertices.size(), vertices.data(), GL_STATIC_DRAW);
		//
		//	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo_wire_sphere.mesh_ebo);
		//	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint16_t)*indices.size(), indices.data(), GL_STATIC_DRAW);
		//}
	
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
				//glMemoryBarrier(GL_ALL_BARRIER_BITS); // TODO: Is a barrier needed here?
				glMemoryBarrier(GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT|GL_COMMAND_BARRIER_BIT|GL_ATOMIC_COUNTER_BARRIER_BIT|GL_SHADER_STORAGE_BARRIER_BIT);
				
				glBindVertexArray(indirect_lines_vao);
				glBindBuffer(GL_DRAW_INDIRECT_BUFFER, indirect_vbo);
					
				glDrawArraysIndirect(GL_LINES, (void*)offsetof(IndirectBuffer, lines.cmd));

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
