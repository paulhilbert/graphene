#ifndef GRAPHENE_H
#define GRAPHENE_H

/// begin here ///
#include <iostream>
#include <memory>

#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES
#endif

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
#include <windows.h>
#endif

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <FW/Events/EventHandler.h>
#include <FW/Factory.h>

#include <GUI/Backend.h>
#include <GUI/GUIFactoryHandle.h>

namespace FW {

class Graphene {
	public:
		typedef std::shared_ptr<Graphene> Ptr;
		struct Impl;
		friend class ::GUI::Backend;

	public:
		Graphene(GUI::Backend::Ptr backend, FW::Events::EventHandler::Ptr eventHandler, bool singleMode);
		virtual ~Graphene();

		int run(int fps);

		void addFactory(std::string name, Factory::Ptr factory);
		Factory::Ptr getFactory(std::string name);

	
	protected:
		std::shared_ptr<Impl> m_impl;
};

} // FW

#endif // GRAPHENE_H
