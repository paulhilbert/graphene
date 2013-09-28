#include "Visualizer.h"

#include <chrono>
#include <Testing/asserts.h>
#include <Algorithm/Sets.h>

namespace FW {

Visualizer::Visualizer(std::string id) : m_id(id) {
}

Visualizer::~Visualizer() {
}

std::string Visualizer::id() const {
	return m_id;
}

FW::VisualizerHandle::Ptr Visualizer::fw() {
	asserts(m_fw, "Framework handle not yet available. Do not use Visualizer fw() and gui() methods in Visualizer constructor.");
	return m_fw;
}

GUI::VisualizerHandle::Ptr Visualizer::gui() {
	asserts(m_gui, "Framework handle not yet available. Do not use Visualizer fw() and gui() methods in Visualizer constructor.");
	return m_gui;
}

void Visualizer::execute(Job task, Job finally) {
	m_tasks.push_back(std::make_pair(std::move(std::async(std::launch::async, task)), std::move(finally)));
}

void Visualizer::setHandles(FW::VisualizerHandle::Ptr fw, GUI::VisualizerHandle::Ptr gui) {
	m_fw = fw;
	m_gui = gui;
}

void Visualizer::waitForTasks() {
	Algorithm::remove(m_tasks, [&] (const Task& task) {
		if (task.first.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) return false;
		if (task.second) task.second();
		return true;
	});
}

} // FW
