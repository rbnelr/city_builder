#pragma once
#include "common.hpp"
#include "opengl.hpp"

namespace ogl {
	
struct BindlessTextureManager {
	enum class Type {
		SRGB8,
		SRGB_A8,
	};
	struct _Format { GLenum internal_format, format, type; };
	static constexpr _Format FORMATS[] = {
		{ GL_SRGB8,        GL_RGB,   GL_UNSIGNED_BYTE },
		{ GL_SRGB8_ALPHA8, GL_RGBA,  GL_UNSIGNED_BYTE },
	};
	
	template <typename T>
	static Type get_type (Image<T> const& img);
	template<> static Type get_type<> (Image<kissmath::srgb8> const& img) { return Type::SRGB8; }
	template<> static Type get_type<> (Image<kissmath::srgba8> const& img) { return Type::SRGB_A8; }
	
	class BindlessTexture {
		MOVE_ONLY_CLASS(BindlessTexture)
	public:

		Texture2D texture = {};
		GLuint64  handle = 0; // bindless handle
		
		friend void swap (BindlessTexture& l, BindlessTexture& r) {
			std::swap(l.texture, r.texture);
			std::swap(l.handle, r.handle);
		}

		BindlessTexture () {}
		~BindlessTexture () {
			if (handle)
				glMakeTextureHandleNonResidentARB(handle); // needed?
		}

	};
	struct LoadedTexture {
		Type type;
		int2 size;
		int  mips;

		BindlessTexture tex = {};
	};

	std::vector<LoadedTexture> loaded_textures;
	std::unordered_map<std::string, int> lookup;

	void clear () {
		lookup.clear();
		loaded_textures.clear();
		loaded_textures.shrink_to_fit();
	}
	// TODO: allow removing of textures?

	// Bindless handles have sampler objects baked into them
	// we can't really switch filtering modes on demand, this is not really a problem because usually you want this sampler
	Sampler default_sampler = make_sampler("sampler_normal", FILTER_MIPMAPPED, GL_REPEAT, true);

	// lookup indirection to allow switching out textures (including different sizes)
	// and potentially streaming in/out mipmaps without having to update all instance data
	// instance data can work with texture ids instead of uint64_t handles
	// maybe this is not really needed?
	// another advantage is to avoid needing glVertexAttribLPointer to use uint64_t in vertex data

	int get_tex_id (std::string_view filepath) {
		auto it = lookup.find(std::string(filepath)); // alloc string because unordered_map is dumb
		if (it == lookup.end())
			return 0; // default tex
		return it->second;
	}

	Ssbo bindless_tex_lut = {"bindless_ssbo"};

	// TODO: add flag to allow only calling this when textures are added or removed?
	void update_lut (int ssbo_binding_slot) {
		std::vector<GLuint64> data;
		data.resize(loaded_textures.size());
		for (int i=0; i<(int)data.size(); ++i)
			data[i] = loaded_textures[i].tex.handle;

		glBindBuffer(GL_SHADER_STORAGE_BUFFER, bindless_tex_lut);
		glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(uint64_t)*data.size(), nullptr, GL_STREAM_DRAW);
		glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(uint64_t)*data.size(), data.data(), GL_STREAM_DRAW);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, ssbo_binding_slot, bindless_tex_lut);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	}

	BindlessTextureManager () {
		ZoneScoped;

		// load default texture as id 0
		load_texture<srgb8>("misc/default.png");
	}
	
	// TODO: Currently cannot unload a texture

	// load texture with default sampler (filter=FILTER_MIPMAPPED, wrap_mode=GL_REPEAT, aniso=true)
	template <typename T>
	void load_texture (const char* filepath) {
		load_texture<T>(filepath, default_sampler);
	}

	// load texture with specific sampler (make sure the sampler has sufficient lifetime)
	template <typename T>
	void load_texture (const char* filepath, Sampler& sampler) {
		ZoneScoped;

		auto str = std::string(filepath);

		auto it = lookup.find(str);
		if (it != lookup.end()) {
			fprintf(stderr, "Error! Texture \"%s\" already loaded", filepath);
			assert(false);
			return;
		}

		printf("loading bindless texture \"%s\"...\n", filepath);

		Image<T> img;
		if (!Image<T>::load_from_file(prints("assets/%s", filepath).c_str(), &img)) {
			fprintf(stderr, "Error! Could not load texture \"%s\"", filepath);
			assert(false);
			return;
		}
		
		lookup[std::move(str)] = (int)loaded_textures.size();
		auto& t = loaded_textures.emplace_back();
		
		t.type = get_type(img);
		t.size = img.size;
		t.mips = calc_mipmaps(t.size.x, t.size.y);

		t.tex.texture = {filepath};
		auto form = FORMATS[(int)t.type];

		{
			ZoneScopedN("upload");
			
			glBindTexture(GL_TEXTURE_2D, t.tex.texture);
			glTexStorage2D(GL_TEXTURE_2D, t.mips, form.internal_format, t.size.x, t.size.y);
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0,0, t.size.x, t.size.y, form.format, form.type, img.pixels);
			glGenerateMipmap(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, 0);

			t.tex.handle = glGetTextureSamplerHandleARB(t.tex.texture, sampler);
			glMakeTextureHandleResidentARB(t.tex.handle);
		}
	}

};

} // namespace ogl
