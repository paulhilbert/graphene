#include "Range.h"

namespace GUI {
namespace Property {

Range::Range(std::string label) : Base(), Notify<void (double)>(), Value<double>(), Labeled(label) {
}

Range::~Range() {
}

} // Property
} // GUI
