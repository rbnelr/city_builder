#version 430
#include "common.glsl"

VS2FS Vertex {
	vec2 uv;
	
	flat vec4 text_col;
} v;

#ifdef _VERTEX
	// quad mesh
	layout(location = 0) in vec2 quad; // quad vertex
	// instanced glyph
	layout(location = 1) in vec4 px_rect; // x0,y0, x1,y1
	layout(location = 2) in vec4 uv_rect; // x0,y0, x1,y1
	layout(location = 3) in vec4 col;
	
	void main () {
		vec2 pos_px = mix(px_rect.xy, px_rect.zw, quad);
		vec2 uv     = mix(uv_rect.xy, uv_rect.zw, quad);
		
		vec2 pos = pos_px / view.viewport_size;
		pos.y = 1.0 - pos.y; // y coords are top-down
		
		gl_Position = vec4(pos * 2.0 - 1.0, 0.0, 1.0);
		v.uv = uv;
		v.text_col = col;
	}
#endif
#ifdef _FRAGMENT
	uniform sampler2D atlas_tex;
	
#if !SDF
	out vec4 frag_col;
	void main () {
		float alpha = texture(atlas_tex, v.uv).r;
		frag_col = v.text_col * (v.text_col.a * alpha);
	}
#else
	uniform float sdf_onedge;
	uniform float sdf_scale;
	
	uniform vec4  sdf_outline_col = vec4(0,0,0,1);
	uniform float sdf_outline_w = 3.0;
	uniform float sdf_grow = 0.0;
	
	out vec4 frag_col;
	void main () {
		vec2 fw = fwidth(v.uv) * vec2(textureSize(atlas_tex, 0)); // texels / screen pixel 
		float scale = 2.0 / (fw.x + fw.y);
		
		
		float sdf = ((texture(atlas_tex, v.uv).r - sdf_onedge) * sdf_scale);
		sdf += sdf_grow;
		
		float ialpha = clamp(scale *  sdf                  + 0.5, 0.0, 1.0);
		float oalpha = clamp(scale * (sdf + sdf_outline_w) + 0.5, 0.0, 1.0);
		
		frag_col = sdf_outline_col * (sdf_outline_col.a * (oalpha - ialpha))
		              + v.text_col * (     v.text_col.a *  ialpha          );
	}
#endif

#endif
