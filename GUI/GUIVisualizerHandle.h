#ifndef GUIVISUALIZERHANDLE_H_
#define GUIVISUALIZERHANDLE_H_

#include <memory>

#include "Property/Container.h"
#include "Mode/Handle.h"
#include "GUILog.h"
#include "Status.h"

namespace GUI {

class Backend;

class VisualizerHandle {
	public:
		typedef std::shared_ptr<VisualizerHandle> Ptr;
		typedef std::weak_ptr<VisualizerHandle>   WPtr;

	public:
		VisualizerHandle(Property::Container::Ptr properties, Mode::Handle::Ptr modes, Log::Ptr log, Status::Ptr status);
		virtual ~VisualizerHandle();

		Property::Container::Ptr properties();
		Mode::Handle::Ptr        modes();
		Log::Ptr                 log();
		Status::Ptr              status();

	protected:
		Property::Container::Ptr   m_properties;
		Mode::Handle::Ptr          m_modes;
		Log::Ptr                   m_log;
		Status::Ptr                m_status;
};

} // GUI

#endif /* GUIVISUALIZERHANDLE_H_ */
