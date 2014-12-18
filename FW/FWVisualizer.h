/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef FWVISUALIZER_H_
#define FWVISUALIZER_H_

/**
 *  @file Visualizer.h
 *
 *  @brief Defines base class for custom visualizers
 *
 */

#include <include/common.h>
#include <include/ogl.h>
#include <include/config.h>
#include <future>

#include <FW/FWVisualizerHandle.h>
#include <GUI/GUIVisualizerHandle.h>
#include <FW/FWTask.h>

#include <GUI/GUIProgressBarPool.h>

#include <harmont/harmont.hpp>
#include <harmont/deferred_renderer.hpp>


namespace FW {

class Graphene;

typedef Eigen::AlignedBox<float, 3> BoundingBox;

/**
 *  @brief Base class for custom visualizers
 *
 *  This class serves as an abstract base class for actual visualizers.
 *  Inherited class should define init() and render() member functions.
 *  For examples see the visualizer examples.
 */
class Visualizer {
	public:
		/** shared pointer to this class */
		typedef std::shared_ptr<Visualizer> Ptr;
		/** weak pointer to this class */
		typedef std::weak_ptr<Visualizer> WPtr;
		/** defines a function to process in a separate thread, provided a progress bar */
		friend class Graphene;

	public:
		/**
		 *  Constructor
		 *
		 *  @param id Identifier of visualizer. Inherited classes should ignore this id (delegate to base constructor)
		 */
		Visualizer(std::string id);

		/**
		 *  Destructor
		 */
		virtual ~Visualizer();

		/**
		 *  Returns identifier of visualizer (usually the name)
		 *
		 *  @return Identifier
		 */
		std::string id() const;

		/**
		 *  Returns access handle to framework functionality
		 *
		 *  @return Access handle to framework.
		 *  @see FW::VisualizerHandle
		 */
		FW::VisualizerHandle::Ptr  fw();

		/**
		 *  Returns access handle to GUI functionality
		 *
		 *  @return Access handle to GUI
		 *  @see GUI::VisualizerHandle
		 */
		GUI::VisualizerHandle::Ptr gui();

		/**
		 *  Pure virtual method initializing inherited visualizers.
		 */
		virtual void init() = 0;

		/**
		 *  Returns renderable object given identifier.
		 *
         *  @param identifier Name used when adding this renderable object.
		 *  @return Shared pointer to specified renderable object.
		 */
        harmont::renderable::ptr_t object(std::string identifier);

		/**
		 *  Returns renderable object given identifier.
		 *
         *  @param identifier Name used when adding this renderable object.
		 *  @return Shared pointer to specified renderable const object.
		 */
        harmont::renderable::const_ptr_t object(std::string identifier) const;

        harmont::deferred_renderer::object_map_t objects() const;

        /**
         * Adds renderable object to list of objects.
         *
         * @param identifier Name of object used for later access.
         * @param object Renderable object.
         */
        void addObject(std::string identifier, harmont::renderable::ptr_t object);

        /**
         * Remove previously added renderable object.
         *
         * @param identifier Name of object to remove.
         */
        void removeObject(std::string identifier);

		/**
		 *  Returns task with given id.
		 *
		 *  @Returns Task with given id or nullptr if not existent.
		 */
		Task::Ptr task(Task::Id id);

		/**
		 *  Adds new task and returns shared pointer to it.
		 *
		 *  @param id Task id.
		 *  @param computation Function object to compute when task is run.
		 *  @returns Shared pointer to created task.
		 */
		Task::Ptr addTask(Task::Id id, Task::Computation computation);

		/**
		 *  Adds new task and returns shared pointer to it.
		 *
		 *  @param id Task id.
		 *  @param computation Function object to compute when task is run.
		 *  @returns Shared pointer to created task.
		 */
		Task::Ptr addTask(Task::Id id, Task::IOComputation computation);


	protected:
		void setHandles(FW::VisualizerHandle::Ptr fw, GUI::VisualizerHandle::Ptr gui);
		void setRenderer(harmont::deferred_renderer::ptr_t renderer);
		void setProgressBarPool(GUI::ProgressBarPool::Ptr m_pool);
		void waitForTasks();

	//protected:
		//typedef std::tuple<std::future<void>, Job, GUI::ProgressBar::Ptr> Task;

	protected:
		std::string                        m_id;
		FW::VisualizerHandle::Ptr          m_fw;
		GUI::VisualizerHandle::Ptr         m_gui;
        harmont::deferred_renderer::ptr_t  m_renderer;
		//std::vector<Task>                  m_tasks;
		GUI::ProgressBarPool::Ptr          m_pool;
		std::map<Task::Id, Task::Ptr>      m_tasks;
        std::set<std::string>              m_objects;
};


} // FW

#endif /* FWVISUALIZER_H_ */
