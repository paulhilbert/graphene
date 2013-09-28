#ifndef EVENTSVISUALIZERHANDLE_H_
#define EVENTSVISUALIZERHANDLE_H_

#include <memory>
#include "EventHandler.h"

namespace FW {

class VisualizerHandle;

namespace Events {

class Handle {
	public:
		typedef std::shared_ptr<Handle> Ptr;
		typedef std::weak_ptr<Handle>   WPtr;
		friend class ::FW::VisualizerHandle;

	protected:
		Handle(std::string id, EventHandler::Ptr eventHandler);

	public:
		virtual ~Handle();

		template <class Sig>
		void connect(EventType type, std::function<Sig> receiver);
		void disconnect(EventType type = "__all__");

	protected:
		std::string       m_id;
		EventHandler::Ptr m_eventHandler;
};

#include "Handle.inl"

} // Events
} // FW


#endif /* EVENTSVISUALIZERHANDLE_H_ */
