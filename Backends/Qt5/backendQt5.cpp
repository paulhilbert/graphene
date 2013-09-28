#include <memory>
#include <boost/extension/extension.hpp>

#include "GUI/Qt5Backend.h"

using namespace GUI;

extern "C"
Backend* BOOST_EXTENSION_EXPORT_DECL getBackend() {
	Qt5Backend* be = new Qt5Backend();
	return dynamic_cast<Backend*>(be);
}
