/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "PropBase.h"

namespace GUI {
namespace Property {

Base::Base() : GUIElement() {
}

Base::~Base() {
}

bool Base::isContainer() const {
	return false;
}

} // Property

} // GUI
