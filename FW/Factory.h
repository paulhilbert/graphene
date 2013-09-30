#ifndef FWFACTORY_H_
#define FWFACTORY_H_

#include <memory>
#include <boost/extension/extension.hpp>

#include "Visualizer.h"

#include <GUI/GUIFactoryHandle.h>

namespace FW {

class Factory {
	public:
		typedef std::shared_ptr<Factory> Ptr;
		typedef std::weak_ptr<Factory> WPtr;

	public:
		Factory();
		virtual ~Factory();

		void setGUIHandle(GUI::FactoryHandle::Ptr handle);

		GUI::FactoryHandle::Ptr gui();

		virtual void init() = 0;
		virtual Visualizer::Ptr addVisualizer() = 0;

	protected:
		GUI::FactoryHandle::Ptr m_gui;
};

} // FW

#define VIS_DLL_EXPORT(VIS) \
extern "C" \
FW::Factory* BOOST_EXTENSION_EXPORT_DECL getFactory() { \
	FW::Factory* factory = new VIS::Factory(); \
	return factory; \
}


#endif /* FWFACTORY_H_ */
