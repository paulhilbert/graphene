#include "Button.h"

namespace GUI {
namespace Property {

Button::Button(std::string label) : Base(), Notify<void (void)>(), Labeled(label) {
}

Button::~Button() {
}

} // Property
} // GUI
