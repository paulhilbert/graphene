#include "Color.h"

namespace GUI {
namespace Property {

Color::Color(std::string label) : Base(), Notify<Eigen::Vector4f>(), Value<Eigen::Vector4f>(), Labeled(label) {
}

Color::~Color() {
}

} // Property
} // GUI
