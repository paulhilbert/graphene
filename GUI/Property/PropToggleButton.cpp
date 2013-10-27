/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "PropToggleButton.h"

namespace GUI {
namespace Property {

ToggleButton::ToggleButton(std::string label) : Base(), Notify<void (bool)>(), Value<bool>(), Labeled(label) {
}

ToggleButton::~ToggleButton() {
}

} // Property
} // GUI
