#include "Folder.h"

namespace GUI {
namespace Property {

Folder::Folder(std::string label) : Base(), Notify<fs::path>(), Value<fs::path>(), Labeled(label) {
}

Folder::~Folder() {
}

} // Property
} // GUI
