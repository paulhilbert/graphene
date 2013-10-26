/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Number.h"

namespace GUI {
namespace Property {

Number::Number(std::string label) : Base(), Notify<void (double)>(), Value<double>(), Labeled(label) {
}

Number::~Number() {
}

} // Property
} // GUI
