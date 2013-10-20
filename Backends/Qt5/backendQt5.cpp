#include <include/common.h>
#include <include/ogl.h>
#include <boost/extension/extension.hpp>

#include "GUI/Qt5Backend.h"

using namespace GUI;

extern "C"
BOOST_EXTENSION_EXPORT_DECL Backend* getBackend() {
	Qt5Backend* be = new Qt5Backend();
	return dynamic_cast<Backend*>(be);
}
