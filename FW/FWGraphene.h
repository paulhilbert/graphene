/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef GRAPHENE_H
#define GRAPHENE_H

/**
 *  @internal @file FWGraphene.h
 *
 *  @brief Main framework class of graphene.
 *
 */

#include <include/common.h>
#include <harmont/harmont.hpp>
#include <harmont/deferred_renderer.hpp>

#include <FW/Events/EventsEventHandler.h>
#include <FW/FWFactory.h>

#include <GUI/GUIBackend.h>
#include <GUI/GUIFactoryHandle.h>

namespace FW {

/**
 *  @internal Graphene
 *
 *  @brief Main framework class of graphene.
 */
class Graphene {
	public:
		typedef std::shared_ptr<Graphene> Ptr;
		struct Impl;
		friend class ::GUI::Backend;

	public:
		/** 
		 *  Constructor.
		 *
		 *  @param backend Shared pointer to backend main class.
		 *  @param eventHandler Shared pointer to main event management class.
		 *  @param singleMode Start in single visualizer mode.
		 *  @param noEffects If true, disables render effects.
		 */
		Graphene(GUI::Backend::Ptr backend, FW::Events::EventHandler::Ptr eventHandler, bool singleMode, bool noEffects, std::string hdrPath);

		/**
		 *  Destructor.
		 */
		virtual ~Graphene();

		/**
		 *  Run main application.
		 *
		 *  @param fps Frames per second to render in.
		 *  @return Exit code of application run.
		 */
		int run(int fps);

		/**
		 *  Adds factory.
		 *
		 *  @param name Name to be used for factory.
		 *  @param factory Shared pointer to factory.
		 */
		void addFactory(std::string name, Factory::Ptr factory);

		/**
		 *  Returns factory with given name
		 *
		 *  @param name Name of factory to return.
		 */
		Factory::Ptr getFactory(std::string name);

	
	protected:
		std::shared_ptr<Impl> m_impl;
};

} // FW

#endif // GRAPHENE_H
