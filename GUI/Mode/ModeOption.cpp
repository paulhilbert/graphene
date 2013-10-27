/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "ModeOption.h"

namespace GUI {
namespace Mode {

Option::Option(std::string id, std::string label, fs::path icon) : GUIElement(), m_id(id), m_label(label), m_icon(icon) {
}

Option::~Option() {
}

} // Mode
} // GUI
