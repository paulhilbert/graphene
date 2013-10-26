/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Folder.h"

namespace GUI {
namespace Property {

Folder::Folder(std::string label) : Base(), Notify<void (fs::path)>(), Value<fs::path>(), Labeled(label) {
}

Folder::~Folder() {
}

} // Property
} // GUI
