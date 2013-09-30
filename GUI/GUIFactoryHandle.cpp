#include "GUIFactoryHandle.h"

namespace GUI {

FactoryHandle::FactoryHandle(Property::Container::Ptr properties) : m_properties(properties) {
}

FactoryHandle::~FactoryHandle() {
}

Property::Container::Ptr FactoryHandle::properties() {
	return m_properties;
}

} // GUI
