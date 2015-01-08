/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef FWVISUALIZERHANDLE_H_
#define FWVISUALIZERHANDLE_H_

/**
 *  @file FWVisualizerHandle.h
 *
 *  @brief Defines access class to framework functionality provided to visualizers.
 */

#include <include/common.h>

#include <FW/Events/EventsHandle.h>
#include <FW/Events/EventsModifier.h>
#include <FW/Events/EventsKeys.h>
using FW::Events::Keys;

#include <harmont/camera.hpp>

namespace FW {

class Graphene;


/**
 *  Access class to framework functionality provided to visualizers.
 *
 *  In any visualizers (i.e. classes inheriting from Visualizer)
 *  this handle can be accessed using the fw() member function.
 *
 *  @see GUI::VisualizerHandle
 */
class VisualizerHandle {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<VisualizerHandle> Ptr;
		/** Weak pointer to this class */
		typedef std::weak_ptr<VisualizerHandle>   WPtr;
		friend class Graphene;

	protected:
		VisualizerHandle(std::string id, Events::EventHandler::Ptr eventHandler, harmont::camera::const_ptr camera);

	public:
		/**
		 *  Destructor
		 */
		virtual ~VisualizerHandle();

		/**
		 *  Returns access class to event management system.
		 *
		 *  @see Events::Handle
		 */
		Events::Handle::Ptr    events();

		/**
		 *  Returns access class to keyboard modifier states.
		 *
		 *  @see Events::Modifier
		 */
		Events::Modifier::Ptr  modifier();

        /**
         * @brief Read access to current camera.
         *
         * @return Const shared pointer to current camera object.
         */
        harmont::camera::const_ptr camera() const;

	protected:
		std::string                     m_id;
		Events::Handle::Ptr             m_events;
		Events::Modifier::Ptr           m_modifier;
        harmont::camera::const_ptr      m_camera;
};

} // FW

#endif /* FWVISUALIZERHANDLE_H_ */
