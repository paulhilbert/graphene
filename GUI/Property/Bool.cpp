#include "Bool.h"

namespace GUI {
namespace Property {

Bool::Bool(std::string label) : Base(), Value<bool>(), Notify<void (bool)>(), Labeled(label) {
}

Bool::~Bool() {
}

} // Property
} // GUI
