#ifndef GRAPHENE_H
#define GRAPHENE_H

/**
 *  @internal @file FWGraphene.h
 *
 *  @brief Main framework class of graphene.
 *
 */

#include <include/common.h>
#include <include/ogl.h>

#include <FW/Events/EventHandler.h>
#include <FW/Factory.h>

#include <GUI/Backend.h>
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
		 */
		Graphene(GUI::Backend::Ptr backend, FW::Events::EventHandler::Ptr eventHandler, bool singleMode);

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
