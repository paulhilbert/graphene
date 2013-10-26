/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


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
