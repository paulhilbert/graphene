#include "Files.h"

namespace GUI {
namespace Property {

Files::Files(std::string label) : Base(), Notify<Paths>(), Value<Paths>(), Labeled(label) {
}

Files::~Files() {
}

} // Property
} // GUI
