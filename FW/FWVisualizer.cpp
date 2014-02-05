/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "FWVisualizer.h"

namespace FW {

Visualizer::Visualizer(std::string id) : m_id(id) {
}

Visualizer::~Visualizer() {
}

std::string Visualizer::id() const {
	return m_id;
}

FW::VisualizerHandle::Ptr Visualizer::fw() {
	if (!m_fw) throw std::runtime_error("Framework handle not yet available. Do not use Visualizer fw() and gui() methods in Visualizer constructor.");
	return m_fw;
}

GUI::VisualizerHandle::Ptr Visualizer::gui() {
	if (!m_gui) throw std::runtime_error("Framework handle not yet available. Do not use Visualizer fw() and gui() methods in Visualizer constructor.");
	return m_gui;
}

bool Visualizer::isHDR() const {
	return false;
}

Task::Ptr Visualizer::task(Task::Id id) {
	if (m_tasks.find(id) == m_tasks.end()) return nullptr;
	return m_tasks[id];
}

Task::Ptr Visualizer::addTask(Task::Id id, Task::Computation computation) {
	auto task = std::make_shared<Task>(id, computation);
	m_tasks[id] = task;
	return task;
}

//void Visualizer::execute(Job task, Job finally) {
	//m_tasks.push_back(std::make_tuple(std::move(std::async(std::launch::async, task)), std::move(finally), GUI::ProgressBar::Ptr()));
//}

//void Visualizer::execute(JobWithBar task, Job finally, std::string taskName, int steps) {
	//if (!m_pool) return;
	//auto bar = m_pool->create(taskName, steps);
	//m_tasks.push_back(std::make_tuple(std::async(std::launch::async, task, [&] (int done, int todo) { bar->poll(done, todo); }), finally, bar));
//}

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

void Visualizer::setProgressBarPool(GUI::ProgressBarPool::Ptr pool) {
	m_pool = pool;
}

void Visualizer::waitForTasks() {
	for (auto& t : m_tasks) {
		t.second->poll();
	}
	//Algorithm::remove(m_tasks, [&] (const Task& task) {
		//if (std::get<0>(task).wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) return false;
		//if (std::get<1>(task)) std::get<1>(task)();
		//return true;
	//});
}

} // FW
