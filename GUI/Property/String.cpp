#include "String.h"

namespace GUI {
namespace Property {

String::String(std::string label) : Base(), Notify<std::string>(), Value<std::string>(), Labeled(label) {
}

String::~String() {
}

} // Property
} // GUI
