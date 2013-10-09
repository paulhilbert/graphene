#include "CloudEditor.h"

namespace FW {

CloudEditor::CloudEditor(std::string id, const Paths& paths) : Visualizer(id), PointCloudEditor(id, paths) {
}

CloudEditor::~CloudEditor() {
}

void CloudEditor::init() {
	PointCloudEditor::init(rgbaCyan(0.4f));
	addProperties();
	registerEvents();
}

void CloudEditor::render() {
	PointCloudEditor::render();
}

void CloudEditor::addProperties() {
}

void CloudEditor::registerEvents() {
}

CloudEditor::Factory::Factory() : FW::Factory() {
}

CloudEditor::Factory::~Factory() {
}

void CloudEditor::Factory::init() {
	gui()->properties()->add<Files>("Input Files", "paths")->setExtensions({"pcd"});;
}

Visualizer::Ptr CloudEditor::Factory::addVisualizer() {
	std::string name = gui()->properties()->get<String>({"__name__"})->value();
	auto paths = gui()->properties()->get<Files>({"paths"})->value();
	CloudEditor::Ptr vis(new CloudEditor(name, paths));
	return std::dynamic_pointer_cast<Visualizer>(vis);
}

} // FW

