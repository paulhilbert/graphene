#include "Files.h"

namespace GUI {
namespace Property {

Files::Files(std::string label) : Base(), Notify<void (Paths)>(), Value<Paths>(), Labeled(label) {
}

Files::~Files() {
}

} // Property
} // GUI
