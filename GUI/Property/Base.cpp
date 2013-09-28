#include "Base.h"

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
