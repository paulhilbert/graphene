#include "Option.h"

namespace GUI {
namespace Mode {

Option::Option(std::string id, std::string label, fs::path icon) : GUIElement(), m_id(id), m_label(label), m_icon(icon) {
}

Option::~Option() {
}

} // Mode
} // GUI
