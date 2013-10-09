#include "Skeleton.h"

namespace FW {

Skeleton::Skeleton(std::string id) : _CONSTRUCT_ {
}

Skeleton::~Skeleton() {
}

void Skeleton::init() {
	addProperties();
	registerEvents();
}

void Skeleton::render() {
}

void Skeleton::addProperties() {
}

void Skeleton::registerEvents() {
}

Skeleton::Factory::Factory() : FW::Factory() {
}

Skeleton::Factory::~Factory() {
}

void Skeleton::Factory::init() {
}

Visualizer::Ptr Skeleton::Factory::addVisualizer() {
	std::string name = gui()->properties()->get<String>({"__name__"})->value();
	Skeleton::Ptr vis(new Skeleton(name));
	return std::dynamic_pointer_cast<Visualizer>(vis);
}

} // FW

