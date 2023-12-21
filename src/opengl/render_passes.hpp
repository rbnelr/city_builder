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
	
	template <typename T>
	void load (const char* filename) {
		ZoneScoped;

		bindless_textures.load_texture<T>(filename);
	}
	
	template <typename T>
	void load (DiffNorm& df) {
		ZoneScoped;

		bindless_textures.load_texture<T>(df.diffuse);
		bindless_textures.load_texture<T>(df.normal);
	}

	Textures () {
		ZoneScoped;

		// TODO: don't hardcode these, rather put them into assets and have used assets trigger load of their textures

		load<srgba8>("misc/line.png"          );
		load<srgba8>("misc/stripe.png"        );
		load<srgba8>("misc/shark_teeth.png"   );
		
		for (auto& fn : turn_arrows)
			load<srgba8>(fn);
		
		load<srgb8>(asphalt);
		load<srgb8>(pavement);
		load<srgb8>(curb);

		//bindless_textures.load_texture<srgba8>("misc/turn_arrow_LSR.png");
	}

	//Sampler sampler_heightmap = make_sampler("sampler_heightmap", FILTER_BILINEAR,  GL_REPEAT);
	Sampler sampler_normal = make_sampler("sampler_normal", FILTER_MIPMAPPED, GL_REPEAT, true);


	//Texture2D clouds = load_texture<srgba8>("clouds", "textures/clouds.png");
	Texture2D grid = load_texture<srgba8>("grid", "misc/grid2.png");
	Texture2D terrain_diffuse = load_texture<srgb8>("terrain_diffuse", "misc/Rock_Moss_001_SD/Rock_Moss_001_basecolor.jpg");
	
	Texture2D house_diff = load_texture<srgb8>("house_Diff", "buildings/house.png");
	Texture2D streetlight_diff = load_texture<srgb8>("streetlight_Diff", "props/streetlight_Diff.png");
	Texture2D car_diff = load_texture<srgb8>("car_Diffe", "cars/car.png");
	
	Texture2D cracks = load_texture<srgb8>("cracks", "misc/cracks.png"); // TODO: support single channel

	template <typename T>
	static Texture2D load_texture (std::string_view gl_label, const char* filepath) {
		ZoneScoped;

		Texture2D tex = {gl_label};
		if (!upload_texture2D<T>(tex, prints("assets/%s", filepath).c_str()))
			assert(false);
		return tex;
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

	Render_Texture depth  = {};
	Render_Texture col    = {};
	Render_Texture norm   = {};

	Sampler sampler = make_sampler("gbuf_sampler", FILTER_NEAREST, GL_CLAMP_TO_EDGE);

	void resize (int2 size) {
		glActiveTexture(GL_TEXTURE0);
		
		depth  = Render_Texture("gbuf.depth", size, depth_format);
		col    = Render_Texture("gbuf.col",   size, col_format);
		norm   = Render_Texture("gbuf.norm",  size, norm_format);
		
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

	template <typename FUNC>
	void draw_cascades (App& app, StateManager& state, FUNC draw_scene) {
		
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
			float3 center = app.view.cam_pos;
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
			ATTRIB(FLT3, Vertex, pos),
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
			ATTRIB(FLT3, Instance, pos),
			ATTRIB(FLT,  Instance, rot),
			ATTRIB(FLT3, Instance, size),
			ATTRIB(INT,  Instance, tex_id),
			ATTRIB(FLT2, Instance, uv_scale),
			ATTRIB(FLT4, Instance, col),
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

	struct MeshInstance {
		//int    type;
		float3 pos;
		float  radius;
		float3 col;

		VERTEX_CONFIG(
			ATTRIB(FLT3, MeshInstance, pos),
			ATTRIB(FLT , MeshInstance, radius),
			ATTRIB(FLT3, MeshInstance, col),
		)
	};

	// All meshes/lods vbo (indexed) GL_ARRAY_BUFFER / GL_ELEMENT_ARRAY_BUFFER
	// All entities instance data GL_ARRAY_BUFFER
	VertexBufferInstancedI vbo = vertex_buffer_instancedI<vert_t, MeshInstance>("point_lights");

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

	template <typename FUNC>
	void update_instances (FUNC get_instances) {
		ZoneScoped;
		OGL_TRACE("update lights");

		auto instances = get_instances();
		vbo.stream_instances(instances);

		instance_count = (GLsizei)instances.size();
	}

	void draw (StateManager& state, Gbuffer& gbuf) {
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

			state.bind_textures(shad, {
				{ "gbuf_depth", { GL_TEXTURE_2D, gbuf.depth }, gbuf.sampler },
				{ "gbuf_col",   { GL_TEXTURE_2D, gbuf.col   }, gbuf.sampler },
				{ "gbuf_norm",  { GL_TEXTURE_2D, gbuf.norm  }, gbuf.sampler },
			});

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
	
	Shader* shad_fullscreen_lighting = g_shaders.compile("fullscreen_lighting");
	Shader* shad_postprocess = g_shaders.compile("postprocess");
	
	float exposure = 1.0f;
	
	void imgui () {
		renderscale.imgui();

		shadowmap.imgui();

		if (ImGui::TreeNode("Postprocessing")) {
			ImGui::SliderFloat("exposure", &exposure, 0.02f, 20.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
			ImGui::TreePop();
		}
	}
	
	void update (int2 window_size) {
		ZoneScoped;

		shadowmap.update();

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

			PipelineState s;
			s.depth_test = false;
			s.depth_write = false;
			s.blend_enable = false;
			state.set(s);
			
			glUseProgram(shad_fullscreen_lighting->prog);

			shad_fullscreen_lighting->set_uniform("shadowmap_mat", shadowmap.view_casc0.world2clip);
			shad_fullscreen_lighting->set_uniform("shadowmap_dir", (float3x3)shadowmap.view_casc0.cam2world * float3(0,0,-1));
			shad_fullscreen_lighting->set_uniform("shadowmap_bias_fac", shadowmap.bias_fac);
			shad_fullscreen_lighting->set_uniform("shadowmap_bias_max", shadowmap.bias_max);
			shad_fullscreen_lighting->set_uniform("shadowmap_cascade_factor", shadowmap.cascade_factor);
			
			state.bind_textures(shad_fullscreen_lighting, {
				{ "gbuf_depth", { GL_TEXTURE_2D, gbuf.depth }, gbuf.sampler },
				{ "gbuf_col",   { GL_TEXTURE_2D, gbuf.col   }, gbuf.sampler },
				{ "gbuf_norm",  { GL_TEXTURE_2D, gbuf.norm  }, gbuf.sampler },

				{ "shadowmap", { GL_TEXTURE_2D_ARRAY, shadowmap.shadow_tex }, shadowmap.sampler },
				{ "shadowmap2", { GL_TEXTURE_2D_ARRAY, shadowmap.shadow_tex }, shadowmap.sampler2 },
				
				{"grid_tex", texs.grid, texs.sampler_normal},
			});

			draw_fullscreen_triangle(state);
		}

		light_renderer.draw(state, gbuf);
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
