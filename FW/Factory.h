/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef FWFACTORY_H_
#define FWFACTORY_H_

/**
 *  @file Factory.h
 *
 *  @brief Contains base factory class.
 *
 */

#include <include/common.h>
#include <boost/extension/extension.hpp>

#include "Visualizer.h"

#include <GUI/GUIFactoryHandle.h>

namespace FW {


/** 
 *  @brief Base factory class.
 *
 *  Custom factory classes should inherit from this class.
 *  The pattern for own visualizers looks like:
 *
 *      class CustomVisualizer : public Visualizer {
 *          public:
 *              typedef std::shared_ptr<Skeleton> Ptr;
 *              typedef std::weak_ptr<Skeleton>   WPtr;
 *
 *          public:
 *              class Factory;
 *              
 *          ...
 *      };
 *
 *      class CustomVisualizer::Factory : public Factory {
 *          ...
 *      };
 *
 *  Any inherited classes shall implement the init method
 *  (most often one simply adds properties using the gui() handle during init)
 *  and the addVisualizer method. The pattern for the latter is:
 *
 *      Visualizer::Ptr CustomVis::Factory::addVisualizer() {
 *          std::string name = gui()->properties()->get<String>({"__name__"})->value();
 *          ... get additional property values...
 *          CustomVis::Ptr vis(new CustomVis(name, ...additional parameters...));
 *          return std::dynamic_pointer_cast<Visualizer>(vis);
 *      }
 *
 *  @see Visualizer
 */
class Factory {
	public:
		typedef std::shared_ptr<Factory> Ptr;
		typedef std::weak_ptr<Factory> WPtr;

	public:
		/**
		 *  Constructor.
		 */
		Factory();

		/**
		 *  Destructor.
		 */
		virtual ~Factory();

		/**
		 *  @internal setGUIHandle(GUI::FactoryHandle::Ptr handle)
		 *
		 *  @brief Internal method to set the gui handle accessible through gui().
		 *
		 *  @see gui() to access this handle
		 */
		void setGUIHandle(GUI::FactoryHandle::Ptr handle);

		/**
		 *  @brief Get access handle to GUI part of framework.
		 *
		 *  Usually this method is used inside init() in order to
		 *  add properties to the "Add Visualizer" dialog.
		 *
		 *  @see GUI::FactoryHandle
		 */
		GUI::FactoryHandle::Ptr gui();

		/**
		 *  @brief Abstract method inherited by factory implementations
		 */
		virtual void init() = 0;

		/**
		 *  @brief Abstract method inherited by factory implementations
		 */
		virtual Visualizer::Ptr addVisualizer() = 0;

	protected:
		GUI::FactoryHandle::Ptr m_gui;
};

} // FW

/**
 *  @brief Define plugin interface method.
 *
 *  Macro used to automagically define the getFactory() method
 *  used by the framework plugin mechanism to load the factory.
 */
#define VIS_DLL_EXPORT(VIS) \
extern "C" \
BOOST_EXTENSION_EXPORT_DECL FW::Factory* getFactory() { \
	FW::Factory* factory = new VIS::Factory(); \
	return factory; \
}


#endif /* FWFACTORY_H_ */
