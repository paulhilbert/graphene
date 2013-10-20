#include "Visualizer.h"

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

void Visualizer::execute(JobWithBar task, Job finally, std::string taskName, int steps) {
	if (!m_pool) return;
	auto bar = m_pool->create(taskName, steps);
	m_tasks.push_back(std::make_pair(std::async(std::launch::async, task, bar), finally));
}

std::vector<std::string> Visualizer::path(std::string&& s0) {
	std::vector<std::string> p(1, s0);
	return p;
}

std::vector<std::string> Visualizer::path(std::string&& s0, std::string&& s1) {
	std::vector<std::string> p(1, s0);
	p.push_back(s1);
	return p;
}

std::vector<std::string> Visualizer::path(std::string&& s0, std::string&& s1, std::string&& s2) {
	std::vector<std::string> p(1, s0);
	p.push_back(s1);
	p.push_back(s2);
	return p;
}

//void Visualizer::execute(JobWithPool task, Job finally) {
//	task(m_pool);
//	if (finally) finally();
//	//m_tasks.push_back(std::make_pair(std::move(std::async(std::launch::async, task, m_pool)), std::move(finally)));
//}

void Visualizer::setHandles(FW::VisualizerHandle::Ptr fw, GUI::VisualizerHandle::Ptr gui) {
	m_fw = fw;
	m_gui = gui;
}

void Visualizer::setProgressBarPool(IO::AbstractProgressBarPool::Ptr pool) {
	m_pool = pool;
}

void Visualizer::waitForTasks() {
	Algorithm::remove(m_tasks, [&] (const Task& task) {
		if (task.first.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) return false;
		if (task.second) task.second();
		return true;
	});
}

} // FW
