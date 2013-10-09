#include "Modifier.h"

namespace FW {

Modifier::Modifier(std::string id) : Visualizer(id) {
}

Modifier::~Modifier() {
}

void Modifier::init() {
	addProperties();
	registerEvents();
	printStatus();
}

void Modifier::render() {

}

void Modifier::addProperties() {
}

void Modifier::registerEvents() {
	// we use the event type POST_EVENT to update after *any* event occured
	fw()->events()->connect<void (std::string)>("POST_EVENT", [&] (std::string eventType) { printStatus(); });
}

void Modifier::printStatus() {
	std::string status  = std::string("shift: ")  + (fw()->modifier()->shift() ? "down" : "  up");
					status += std::string(" | ctrl: ")  + (fw()->modifier()->ctrl() ? "down" : "  up");
					status += std::string(" | alt: ")   + (fw()->modifier()->alt() ? "down" : "  up");
					status += std::string(" | altgr: ") + (fw()->modifier()->altgr() ? "down" : "  up");
	gui()->status()->set(status);
}

Modifier::Factory::Factory() : FW::Factory() {
}

Modifier::Factory::~Factory() {
}

void Modifier::Factory::init() {
}

Visualizer::Ptr Modifier::Factory::addVisualizer() {
	std::string name = gui()->properties()->get<String>({"__name__"})->value();
	Modifier::Ptr vis(new Modifier(name));
	return std::dynamic_pointer_cast<Visualizer>(vis);
}

} // FW

