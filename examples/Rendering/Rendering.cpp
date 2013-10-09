#include "Rendering.h"

#include <Vis/Colors.h>
using namespace Vis::Colors;

namespace FW {

Rendering::Rendering(std::string id) : Visualizer(id), m_colX(Eigen::Vector4f(1.f, 0.f, 0.f, 1.f)), m_colY(Eigen::Vector4f(0.f, 1.f, 0.f, 1.f)), m_colZ(Eigen::Vector4f(0.f, 0.f, 1.f, 1.f)) {
}

Rendering::~Rendering() {
}

void Rendering::init() {
	addProperties();
	m_lines = Rendered::Lines::Ptr(new Rendered::Lines(rgbaWhite()));
	std::vector<Eigen::Vector3f> points;
	points.push_back(Eigen::Vector3f(0.f, 0.f, 0.f));
	points.push_back(Eigen::Vector3f(1.f, 0.f, 0.f));
	points.push_back(Eigen::Vector3f(0.f, 0.f, 0.f));
	points.push_back(Eigen::Vector3f(0.f, 1.f, 0.f));
	points.push_back(Eigen::Vector3f(0.f, 0.f, 0.f));
	points.push_back(Eigen::Vector3f(0.f, 0.f, 1.f));
	m_lines->set(points);
	colorize();
}

void Rendering::render() {
	m_lines->render(fw()->transforms()->modelview(), fw()->transforms()->projection());
}

void Rendering::addProperties() {
	auto colX = gui()->properties()->add<Color>("X-Axis: ");
	colX->setValue(m_colX);
	colX->setCallback([&] (Eigen::Vector4f color) { m_colX = color; colorize(); });
	auto colY = gui()->properties()->add<Color>("Y-Axis: ");
	colY->setValue(m_colY);
	colY->setCallback([&] (Eigen::Vector4f color) { m_colY = color; colorize(); });
	auto colZ = gui()->properties()->add<Color>("Z-Axis: ");
	colZ->setValue(m_colZ);
	colZ->setCallback([&] (Eigen::Vector4f color) { m_colZ = color; colorize(); });
}

void Rendering::colorize() {
	m_lines->annotate({0, 1}, "xAxis")->colorize(m_colX);
	m_lines->annotate({2, 3}, "yAxis")->colorize(m_colY);
	m_lines->annotate({4, 5}, "zAxis")->colorize(m_colZ);
}

Rendering::Factory::Factory() : FW::Factory() {
}

Rendering::Factory::~Factory() {
}

void Rendering::Factory::init() {
}

Visualizer::Ptr Rendering::Factory::addVisualizer() {
	std::string name = gui()->properties()->get<String>({"__name__"})->value();
	Rendering::Ptr vis(new Rendering(name));
	return std::dynamic_pointer_cast<Visualizer>(vis);
}

} // FW

