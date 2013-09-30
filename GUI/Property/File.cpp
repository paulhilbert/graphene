#include "File.h"

namespace GUI {
namespace Property {

File::File(std::string label) : Base(), Notify<void (fs::path)>(), Value<fs::path>(), Labeled(label) {
}

File::~File() {
}

} // Property
} // GUI
