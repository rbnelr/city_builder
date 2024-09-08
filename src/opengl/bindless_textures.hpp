#pragma once
#include "common.hpp"
#include "opengl.hpp"

namespace ogl {
	
struct _TexFormat { GLenum internal_format, format, type; };

// TODO: actually handle non-linear color maps?
template <typename T>
inline constexpr _TexFormat get_format ();
template<> inline constexpr _TexFormat get_format<srgb8 > () { return { GL_SRGB8,        GL_RGB,   GL_UNSIGNED_BYTE }; }
template<> inline constexpr _TexFormat get_format<srgba8> () { return { GL_SRGB8_ALPHA8, GL_RGBA,  GL_UNSIGNED_BYTE }; }

struct BindlessTextureManager {
	
	// Seperate Move only class that handles just the resources it needs to avoid manually swapping all sorts of members
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

	// Do not track texture type, size and mip count since textures currently follow model of
	//  create entry (assign id to name)
	//  upload textures to entry, (also remove them and the slot itself in the future ?)
	//  replace texture etc.
	//  query texture id and access on gpu through id
	// (types, size etc. would be needed to do any sort of virtual texturing or similar)
	
	// lookup indirection to allow switching out textures (including different sizes)
	// and potentially streaming in/out mipmaps without having to update all instance data
	// instance data can work with texture ids instead of uint64_t handles
	// another advantage is to avoid needing glVertexAttribLPointer to use uint64_t in vertex data

	struct TextureEntry {
		// generic 4 slots that bindless texture users (uploaders and shaders) can use for PBR maps etc.
		// this is needed because otherwise drawn instanced need multiple texture ids passed to them
		// Avoid forcing semanting meaning on them (0 is albedo, 1 is normal etc.) because that's better handles in other places (?)
		BindlessTexture slots[4];
		float uv_scale = 1;
	};

private:
	std::vector<TextureEntry> _entries;
	std::unordered_map<std::string, int> _lookup;
public:

	// Bindless handles have sampler objects baked into them
	// we can't really switch filtering modes on demand, this is not really a problem because usually you want this sampler
	Sampler default_sampler = make_sampler("sampler_normal", FILTER_MIPMAPPED, GL_REPEAT, true);

	Ssbo bindless_tex_lut = {"bindless_ssbo"};
	
	static constexpr const char* DUMMY_NAME = "<dummy tex>";
	void clear () {
		_lookup.clear();

		_entries.resize(1); // keep dummy tex
		_entries.shrink_to_fit();

		_lookup[DUMMY_NAME] = 0;
	}

	TextureEntry* operator[] (int idx) {
		assert(idx >= 0 || idx < (int)_entries.size());
		return &_entries[idx];
	}
	int operator[] (std::string_view name) {
		auto it = _lookup.find(std::string(name)); // alloc string because unordered_map is dumb
		if (it == _lookup.end())
			return 0; // default tex
		return it->second;
	}
	TextureEntry* get (std::string_view name) {
		int idx = operator[](name);
		if (idx <= 0) return nullptr;
		return &_entries[idx];
	}
	Texture2D* get_gl_tex (std::string_view name, int slot) {
		auto* entry = get(name);
		if (!entry) return nullptr;
		return &entry->slots[slot].texture;
	}

	// TODO: add flag to allow only calling this when textures are added or removed?
	void update_lut (int ssbo_binding_slot) {
		struct Entry {
			GLuint64 handles[4];
			float uv_scale;
			float _pad = {};
		};
		std::vector<Entry> data;
		data.resize(_entries.size());
		for (int i=0; i<(int)data.size(); ++i) {
			auto& tex = _entries[i];
			for (int j=0; j<4; ++j)
				data[i].handles[j] = tex.slots[j].handle;
			data[i].uv_scale = tex.uv_scale;
		}

		glBindBuffer(GL_SHADER_STORAGE_BUFFER, bindless_tex_lut);
		glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(Entry)*data.size(), nullptr, GL_STREAM_DRAW);
		glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(Entry)*data.size(), data.data(), GL_STREAM_DRAW);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, ssbo_binding_slot, bindless_tex_lut);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	}

	BindlessTextureManager () {
		ZoneScoped;
		add_dummy();
	}

	void add_dummy () {
		// load default texture as id 0
		int id = add_entry(DUMMY_NAME);
		assert(id == 0);

		Image<srgb8> img;
		if (!Image<srgb8>::load_from_file("assets/misc/default.png", &img)) {
			assert(false);
		}
		upload_texture(DUMMY_NAME, 0, 0, img, default_sampler, true);
	}
	
	// returns id
	int add_entry (std::string&& name) {
		auto id = this->operator[](name);
		if (id != 0) {
			log_error("Error! Texture \"%s\" already added", name.c_str());
			assert(false);
			return id; // return id anyway to avoid crashes (just overwrites it)
		}

		id = (int)_entries.size();
		_lookup[std::move(name)] = id;
		_entries.emplace_back();
		return id;
	}
	// TODO: Currently cannot unload a texture

	// load texture with default sampler (filter=FILTER_MIPMAPPED, wrap_mode=GL_REPEAT, aniso=true)
	template <typename T>
	void upload_texture (std::string_view name, int slot, Image<T> const& img, bool mips) {
		int id = this->operator[](name);
		upload_texture<T>(name, id, slot, img, default_sampler, mips);
	}

	template <typename T>
	void upload_texture (std::string_view dbg_name, int id, int slot, Image<T> const& img, Sampler& sampler, bool mips) {
		ZoneScoped;
		
		auto form = get_format<T>();
		int num_mips = mips ? calc_mipmaps(img.size.x, img.size.y) : 1;
		
		auto& tex = _entries[id].slots[slot];
		tex.texture = {dbg_name};
		
		{
			ZoneScopedN("upload");
			
			glBindTexture(GL_TEXTURE_2D, tex.texture);
			glTexStorage2D(GL_TEXTURE_2D, num_mips, form.internal_format, img.size.x, img.size.y);
			glTexSubImage2D(GL_TEXTURE_2D, 0, 0,0, img.size.x, img.size.y, form.format, form.type, img.pixels);
			if (num_mips > 1)
				glGenerateMipmap(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, 0);

			tex.handle = glGetTextureSamplerHandleARB(tex.texture, sampler);
			glMakeTextureHandleResidentARB(tex.handle);
		}
	}

};

} // namespace ogl
