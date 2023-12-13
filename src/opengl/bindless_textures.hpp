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

	// Unfortunately bindless handles have sampler objects baked into them
	// so we can't really switch filtering modes on demand, this is not really a problem because usually you want this sampler
	Sampler sampler_normal = make_sampler("sampler_normal", FILTER_MIPMAPPED, GL_REPEAT, true);

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

	BindlessTextureManager () {
		// load default texture as id 0
		load_texture<srgb8>("misc/default.png");
	}

	template <typename T>
	void load_texture (const char* filepath) {

		auto str = std::string(filepath);

		auto it = lookup.find(str);
		if (it != lookup.end()) {
			fprintf(stderr, "Error! Texture \"%s\" already loaded", filepath);
			assert(false);
			return;
		}

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

		glBindTexture(GL_TEXTURE_2D, t.tex.texture);
		glTexStorage2D(GL_TEXTURE_2D, t.mips, form.internal_format, t.size.x, t.size.y);
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0,0, t.size.x, t.size.y, form.format, form.type, img.pixels);
		glGenerateMipmap(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, 0);

		t.tex.handle = glGetTextureSamplerHandleARB(t.tex.texture, sampler_normal);
		glMakeTextureHandleResidentARB(t.tex.handle);

	}

};

} // namespace ogl
