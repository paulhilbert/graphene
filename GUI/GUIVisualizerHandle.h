/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef GUIVISUALIZERHANDLE_H_
#define GUIVISUALIZERHANDLE_H_

/**
 *  @file GUIVisualizerHandle.h
 *
 *  Defines access class to GUI functionality provided to visualizers.
 */

#include <include/common.h>
#include <include/visualizer.h>

#include <GUI/Property/Container.h>
#include <GUI/Mode/Handle.h>
#include "GUILog.h"
#include "Status.h"

namespace GUI {

class Backend;

/**
 *  Access class to GUI for visualizers.
 *
 *  This class can be accessed inside custom visualizers
 *  (i.e. classes inherited from FW::Visualizer) using
 *  the gui() member function.
 */
class VisualizerHandle {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<VisualizerHandle> Ptr;
		/** Weak pointer to this class */
		typedef std::weak_ptr<VisualizerHandle>   WPtr;

	public:
		/**
		 *  Constructor.
		 *
		 *  @param properties Access handle to visualizer property area.
		 *  @param modes Access class to mode management.
		 *  @param log Access class to log facility.
		 *  @param status Access class to status bar.
		 *  @param alwaysActive If true disallows the user to enable/disable visualizer.
		 */
		VisualizerHandle(Property::Container::Ptr properties, Mode::Handle::Ptr modes, Log::Ptr log, Status::Ptr status, bool alwaysActive = false);

		/** Destructor */
		virtual ~VisualizerHandle();

		/**
		 *  Returns access handle to visualizer properties.
		 *
		 *  @return Shared pointer to base property Container.
		 */
		Property::Container::Ptr properties();

		/**
		 *  Returns access handle to mode area.
		 *
		 *  @return Shared pointer to mode management handle.
		 */
		Mode::Handle::Ptr modes();

		/**
		 *  Returns access handle to log facility.
		 *
		 *  @return Shared pointer to log handle.
		 */
		Log::Ptr log();

		/**
		 *  Returns access handle to status bar.
		 *
		 *  @return Shared pointer to status bar handle.
		 */
		Status::Ptr status();

	protected:
		Property::Container::Ptr   m_properties;
		Mode::Handle::Ptr          m_modes;
		Log::Ptr                   m_log;
		Status::Ptr                m_status;
};

} // GUI

#endif /* GUIVISUALIZERHANDLE_H_ */
