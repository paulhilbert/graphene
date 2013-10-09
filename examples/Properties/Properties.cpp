#include "Properties.h"

namespace FW {

Properties::Properties(std::string id) : Visualizer(id) {
}

Properties::~Properties() {
}

void Properties::init() {
	/// bool group ///
	auto groupBool = gui()->properties()->add<Group>("Boolean", "groupBool");

	// bool //
	auto boolean = groupBool->add<Bool>("Test Boolean: ", "bool");
	boolean->setValue(false);
	boolean->setCallback([&](bool value) { print(); });

	// togglebutton //
	auto toggle = groupBool->add<ToggleButton>("Test Toggle", "toggle");
	toggle->setValue(false);
	toggle->setCallback([&](bool value) { print(); });

	/// numeric group ///
	auto groupNumeric = gui()->properties()->add<Group>("Numeric", "groupNumeric");

	// number //
	auto number = groupNumeric->add<Number>("Test Number: ", "number");
	number->setMin(0.2);
	number->setMax(0.6);
	number->setValue(0.5);
	number->setDigits(3);
	number->setStep(0.001);
	number->setCallback([&](double value) { print(); });

	// range //
	auto range = groupNumeric->add<Range>("Test Range: ", "range");
	range->setMin(1.0);
	range->setMax(2.5);
	range->setDigits(2);
	range->setValue(1.0);
	range->setCallback([&](double value) { print(); });

	/// filesystem group ///
	auto sectionFS = gui()->properties()->add<Section>("Filesystem", "sectionFS");
	sectionFS->collapse();

	// open file //
	auto fileOpen = sectionFS->add<File>("Test Open File: ", "fileOpen");
	fileOpen->setMode(File::OPEN);
	fileOpen->setCallback([&] (fs::path value) { gui()->log()->info("Changed file (open) to: "+value.string()); });

	// save file //
	auto fileSave = sectionFS->add<File>("Test Save File: ", "fileSave");
	fileSave->setMode(File::SAVE);
	fileSave->setCallback([&] (fs::path value) { gui()->log()->info("Changed file (save) to: "+value.string()); });

	// files //
	auto files = sectionFS->add<Files>("Test Files: ", "files");
	files->setCallback([&] (Paths value) {
		gui()->log()->info("Changed files to: ");
		for (const auto& p : value) {
			gui()->log()->info(p.string());
		}
	});

	// folder //
	auto folder = sectionFS->add<Folder>("Test Folder: ", "folder");
	folder->setCallback([&] (fs::path value) { gui()->log()->info("Changed directory to: "+value.string()); });


	/// complex group ///
	auto sectionComplex = gui()->properties()->add<Section>("Complex", "sectionComplex");
	sectionComplex->collapse();

	// choice //
	auto choice = sectionComplex->add<Choice>("Test Choice", "choice");
	choice->add("test1", "Test1");
	choice->add("test2", "Test2");
	choice->add("test3", "Test3");
	choice->setCallback([&](std::string option) { print(); });

	// tree //
	auto tree = sectionComplex->add<Tree>("Test Tree", "tree");
	tree->add("id0", {"First"});
	tree->add("id1", {"First", "First"});
	tree->add("id2", {"First", "Second"});
	tree->add("id3", {"First", "Third"});
	tree->add("id4", {"Second"});
	tree->remove("id2");
	tree->setCallback([&] (std::string id, bool state) { gui()->log()->info("Item with id \""+id+"\" is "+(state ? "checked." : "not checked")); });

	// color //
	auto color = sectionComplex->add<Color>("Test Color: ", "color");
	color->setCallback([&] (Eigen::Vector4f value) { print(); });

	// string //
	sectionComplex->add<String>("Test String: ", "string")->setValue("Text to log");

	// button //
	sectionComplex->add<Button>("Test Button")->setCallback([&]() {
		gui()->log()->info("Button pressed. Text in String property is:");
		gui()->log()->info(gui()->properties()->get<String>({"sectionComplex", "string"})->value());
	});

	print();
}

void Properties::render() {
}

void Properties::print() {
	std::stringstream status;
	status.flags( std::ios::showpoint | std::ios::fixed );

	// bool
	bool boolValue = gui()->properties()->get<Bool>({"groupBool", "bool"})->value();
	status << "bool: "; status << (boolValue ? " true" : "false");
	// toggle
	bool toggleValue = gui()->properties()->get<ToggleButton>({"groupBool", "toggle"})->value();
	status << " | toggle: "; status << (toggleValue ? " true" : "false");
	// number
	double numberValue = gui()->properties()->get<Number>({"groupNumeric", "number"})->value();
	status << " | number: "; status.precision(3); status << numberValue;
	// range
	double rangeValue = gui()->properties()->get<Range>({"groupNumeric", "range"})->value();
	status << " | slider: "; status.precision(2); status << rangeValue;
	// choice
	std::string choiceValue = gui()->properties()->get<Choice>({"sectionComplex", "choice"})->value();
	status << " | choice: "; status.precision(2); status << choiceValue;
	// color
	Eigen::Vector4f colorValue = gui()->properties()->get<Color>({"sectionComplex", "color"})->value();
	status << " | color: "; status.precision(2); status << "(" << colorValue[0] << ", " << colorValue[1] << ", " << colorValue[2] << ", " << colorValue[3] << ")";

	
	gui()->status()->set(status.str());
}

Properties::Factory::Factory() : FW::Factory() {
}

Properties::Factory::~Factory() {
}

void Properties::Factory::init() {
}

Visualizer::Ptr Properties::Factory::addVisualizer() {
	std::string name = gui()->properties()->get<String>({"__name__"})->value();
	Properties::Ptr vis(new Properties(name));
	return std::dynamic_pointer_cast<Visualizer>(vis);
}

} // FW

