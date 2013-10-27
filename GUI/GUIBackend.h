/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef BACKEND_H_
#define BACKEND_H_

/**
 *  @internal @file Backend.h
 *
 *  @brief Defines framework's main GUI class.
 *
 */

#include <include/common.h>

#include <FW/Events/EventsEventHandler.h>
#include <FW/Events/EventsModifier.h>

#include "GUIFactoryHandle.h"
#include <FW/FWFactory.h>
#include "GUILog.h"
#include "GUIProgressBarPool.h"

namespace GUI {

/**
 *  @internal Backend
 *
 *  @brief Main GUI class.
 *
 */
class Backend {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Backend> Ptr;
		/** Weak pointer to this class */
		typedef std::weak_ptr<Backend> WPtr;

	public:
		/**
		 *  Constructor
		 */
		Backend();

		/**
		 *  Destructor
		 */
		virtual ~Backend();

		/**
		 *  Pure virtual method initializing the backend.
		 *
		 *  @param argc Argument count supplied to main(int argc, argv* [])
		 *  @param argv Argument array supplied to main(int argc, argv* [])
		 *  @param eventHandler Shared pointer to event management class
		 *  @param singleMode Start in single visualizer mode
		 *  @param verbose Log verbose output
		 */
		virtual void init(int argc, char* argv[], FW::Events::EventHandler::Ptr eventHandler, bool singleMode, bool verbose) = 0;

		/**
		 *  Pure virtual method starting main application thread.
		 *
		 *  @param fps Frames per second to render in.
		 */
		virtual int run(int fps) = 0;

		/**
		 *  Pure virtual method adding a new factory to GUI.
		 *
		 *  @param name Name of factory to add.
		 *  @return Shared pointer to FactoryHandle.
		 */
		virtual FactoryHandle::Ptr addFactory(std::string name) = 0;

		/**
		 *  Pure virtual method adding a new visualizer to GUI.
		 *
		 *  @param name Name of visualizer to add
		 *  @return Shared pointer to VisualizerHandle
		 */
		virtual VisualizerHandle::Ptr addVisualizer(std::string name) = 0;

		/**
		 *  Pure virtual method initializing visualizer in single mode.
		 *
		 *  @return Success state of initialization.
		 */
		virtual bool initSingleVisualizer() = 0;

		/**
		 *  Pure virtual method returning access handle to Log.
		 *
		 *  @return Shared pointer to Log access handle.
		 */
		virtual Log::Ptr getLog() = 0;

		/**
		 *  Pure virtual method returning access handle to status bar.
		 *
		 *  @return Shared pointer to Status bar access handle.
		 */
		virtual Status::Ptr getStatus() = 0;

		/**
		 *  Pure virtual method returning access handle to progress bar pool.
		 *
		 *  @return Shared pointer to progress bar pool access handle.
		 */
		virtual ProgressBarPool::Ptr getProgressBarPool() = 0;

		/**
		 *  Pure virtual method returning access handle to main settings.
		 *
		 *  @return Shared pointer to property Container class.
		 */
		virtual Container::Ptr getMainSettings() = 0;

		/**
		 *  Pure virtual method setting window title.
		 *
		 *  @param title Title to set for main window.
		 */
		virtual void setWindowTitle(std::string title) = 0;

		/**
		 *  Pure virtual method setting window size.
		 *
		 *  @param width New window width.
		 *  @param height New window height.
		 */
		virtual void setWindowSize(int width, int height) = 0;

		/**
		 *  Pure virtual method returning OpenGL drawing area size.
		 *  @return 2D vector representing (width, height) of OpenGL drawing area.
		 */
		virtual Eigen::Vector2i getGLSize() = 0;

		/**
		 *  Pure virtual method returning background color.
		 *
		 *  @return 4D vector returning (R,G,B,A) values of background color.
		 */
		virtual Eigen::Vector4f getBackgroundColor() const = 0;

		/**
		 *  Pure virtual method return active visualizer names.
		 *
		 *  Usually backends should provide for example a checkbox to mark visualizers as active/inactive.
		 *  Implementations of this function should return a vector of visualizer ids representing those
		 *  visualizers marked as active.
		 *
		 *  @return std::vector of active visualizer ids.
		 */
		virtual std::vector<std::string> getActiveVisualizerNames() = 0;

		/* 
		 *  Set function to call when user wants to add a new visualizer.
		 *
		 *  Function parameters are (factoryName, visualizerName).
		 *
		 *  @param onAddVis function to call on visualizer add request.
		 */
		void setAddVisCallback(const std::function<void (std::string, std::string)>& onAddVis);

		/* 
		 *  Set function to call when user wants to remove a visualizer.
		 *
		 *  Function parameter is visualizerName.
		 *
		 *  @param onDelVis function to call on visualizer remove request.
		 */
		void setDelVisCallback(const std::function<void (std::string)>& onDelVis);

		/**
		 *  Set function to call on application exit.
		 *
		 *  @param func Function to call when user requests an application exit.
		 */
		void setExitCallback(std::function<void ()> func);

		/**
		 *  Set function to call when user changes camera control mode.
		 *
		 *  Function parameter is the control mode name.
		 *
		 *  @param onSetCamControl Function to call on mode change.
		 */
		void setCamControlCallback(std::function<void (std::string)> onSetCamControl);

		/**
		 *  Set function to call when user changes camera projection mode.
		 *
		 *  Function parameter is true iff orthogonal projection mode is chosen by user.
		 *
		 *  @param onSetOrtho Function to call on projection mode change.
		 */
		void setOrthoCallback(std::function<void (bool)> onSetOrtho);

		/**
		 *  Pure virtual function setting function to call on render updates.
		 *
		 *  @param func Function to call on render update.
		 */
		virtual void setRenderCallback(std::function<void ()> func) = 0;

#ifdef ENABLE_SCREENCAST
		void setScreencastStartCallback(const std::function<void (fs::path)>& func);
		void setScreencastPauseCallback(const std::function<void (void)>& func);
		void setScreencastResumeCallback(const std::function<void (void)>& func);
		void setScreencastStopCallback(const std::function<void (void)>& func);
#endif // ENABLE_SCREENCAST

	protected:
		std::map<std::string, FW::Factory::WPtr>       m_factories;
		std::function<void (std::string,std::string)>  m_onAddVis;
		std::function<void (std::string)>              m_onDelVis;
		std::function<void ()>                         m_onExit;
		std::function<void (std::string)>              m_onSetCamControl;
		std::function<void (bool)>                     m_onSetOrtho;

#ifdef ENABLE_SCREENCAST
		std::function<void (fs::path)>                 m_onStartScreencast;
		std::function<void (void)>                     m_onPauseScreencast;
		std::function<void (void)>                     m_onResumeScreencast;
		std::function<void (void)>                     m_onStopScreencast;
#endif // ENABLE_SCREENCAST
};

} // GUI

#endif /* BACKEND_H_ */
