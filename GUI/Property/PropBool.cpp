/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "PropBool.h"

namespace GUI {
namespace Property {

Boolean::Boolean(std::string label) : Base(), Value<bool>(), Notify<void (bool)>(), Labeled(label) {
}

Boolean::~Boolean() {
}

} // Property
} // GUI
