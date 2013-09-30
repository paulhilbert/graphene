#include "String.h"

namespace GUI {
namespace Property {

String::String(std::string label) : Base(), Notify<void (std::string)>(), Value<std::string>(), Labeled(label) {
}

String::~String() {
}

} // Property
} // GUI
