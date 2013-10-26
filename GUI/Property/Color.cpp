/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Color.h"

namespace GUI {
namespace Property {

Color::Color(std::string label) : Base(), Notify<void (Eigen::Vector4f)>(), Value<Eigen::Vector4f>(), Labeled(label) {
}

Color::~Color() {
}

} // Property
} // GUI
