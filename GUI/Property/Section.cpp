#include "Section.h"

namespace GUI {
namespace Property {

Section::Section(std::string label) : Container(), Labeled(label) {
}

Section::~Section() {
}

void Section::collapse() {
	setCollapsed(true);
}

void Section::expand() {
	setCollapsed(false);
}

} // Property
} // GUI
