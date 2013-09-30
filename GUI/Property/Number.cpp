#include "Number.h"

namespace GUI {
namespace Property {

Number::Number(std::string label) : Base(), Notify<void (double)>(), Value<double>(), Labeled(label) {
}

Number::~Number() {
}

} // Property
} // GUI
