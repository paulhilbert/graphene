/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "FWVisualizer.h"

namespace FW {

Visualizer::Visualizer(std::string id) : m_id(id), m_renderer(nullptr) {
}

Visualizer::~Visualizer() {
    for (const auto& obj_name : m_objects) {
        removeObject(obj_name);
    }
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

harmont::renderable::ptr_t Visualizer::object(std::string identifier) {
    if (!m_renderer) throw std::runtime_error("Visualizer::object(): No renderer object set for this visualizer."+SPOT);
    return m_renderer->object(identifier);
}

harmont::renderable::const_ptr_t Visualizer::object(std::string identifier) const {
    if (!m_renderer) throw std::runtime_error("Visualizer::object(): No renderer object set for this visualizer."+SPOT);
    return m_renderer->object(identifier);
}

harmont::deferred_renderer::object_map_t Visualizer::objects() const {
    if (!m_renderer) throw std::runtime_error("Visualizer::objects(): No renderer object set for this visualizer."+SPOT);

    harmont::deferred_renderer::object_map_t objs;
    auto graphene_objs = m_renderer->objects();
    for (const auto& obj_name : m_objects) {
        objs[obj_name] = graphene_objs[obj_name];
    }
    return objs;
}

void Visualizer::addObject(std::string identifier, harmont::renderable::ptr_t object) {
    if (!m_renderer) throw std::runtime_error("Visualizer::addObject(): No renderer object set for this visualizer."+SPOT);
    m_renderer->add_object(identifier, object);
    m_objects.insert(identifier);
}

void Visualizer::addObjectGroup(std::string prefix, harmont::renderable_group::ptr_t group) {
    if (!m_renderer) throw std::runtime_error("Visualizer::addObject(): No renderer object set for this visualizer."+SPOT);
    uint32_t idx = 0;
    for (auto& obj : group->objects()) {
        addObject(prefix + std::to_string(idx++), obj);
    }
}

void Visualizer::removeObject(std::string identifier) {
    if (!m_renderer) throw std::runtime_error("Visualizer::removeObject(): No renderer object set for this visualizer."+SPOT);
    auto find_it = m_objects.find(identifier);
    if (find_it == m_objects.end()) throw std::runtime_error("Visualizer::removeObject(): Object \""+identifier+"\" does not exist"+SPOT);
    m_renderer->remove_object(identifier);
    m_objects.erase(find_it);
}

void Visualizer::tryRemoveObject(std::string identifier) {
    if (!m_renderer) throw std::runtime_error("Visualizer::removeObject(): No renderer object set for this visualizer."+SPOT);
    auto find_it = m_objects.find(identifier);
    if (find_it == m_objects.end()) return;
    m_renderer->remove_object(identifier);
    m_objects.erase(find_it);
}

void Visualizer::removeObjectGroup(std::string prefix) {
    uint32_t idx = 0;
    while (m_objects.find(prefix + std::to_string(idx)) != m_objects.end()) {
        removeObject(prefix + std::to_string(idx++));
    }
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

Task::Ptr Visualizer::addTask(Task::Id id, Task::IOComputation computation) {
	auto task = std::make_shared<Task>(id, std::bind(computation, m_pool));
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

//void Visualizer::execute(JobWithPool task, Job finally) {
//	task(m_pool);
//	if (finally) finally();
//	//m_tasks.push_back(std::make_pair(std::move(std::async(std::launch::async, task, m_pool)), std::move(finally)));
//}

void Visualizer::setHandles(FW::VisualizerHandle::Ptr fw, GUI::VisualizerHandle::Ptr gui) {
	m_fw = fw;
	m_gui = gui;
}

void Visualizer::setRenderer(harmont::deferred_renderer::ptr_t renderer) {
    m_renderer = renderer;
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
