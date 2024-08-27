#include "ogl_render.hpp"

namespace ogl {

} // namespace ogl

std::unique_ptr<Renderer> create_ogl_backend () {
	ZoneScoped;
	return std::make_unique<ogl::OglRenderer>();
}
