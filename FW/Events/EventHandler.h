#ifndef FWEVENTHANDLER_H_
#define FWEVENTHANDLER_H_

#include <include/common.h>

#include <Generic/Command.h>
using Generic::Command;
using Generic::BaseCommand;

#include "Modifier.h"


namespace FW {
namespace Events {

typedef std::string EventType;

class Signal {
	public:
		typedef std::shared_ptr<Signal>  Ptr;
		typedef boost::any               Param;
		typedef std::vector<Param>       Params;
		class Impl;

	public:
		Signal(EventType type);

		void           addParam(Param param);

		const Params&  getParams() const;
		EventType      getType() const;

	private:
		std::shared_ptr<Impl> m_impl;
};

class EventHandler {
	public:
		typedef std::shared_ptr<EventHandler>  Ptr;
		class Impl;

	public:
		EventHandler();

		template <class Sig>
		void registerReceiver(EventType eventType, std::string id, std::function<Sig> receiver);

		void unregisterReceiver(EventType eventType, std::string id);
		void unregisterReceiver(std::string id);

		void notify(Signal signal);

		Modifier::Ptr modifier();

		// blocks are ignored by general events and window events (pre/post/resize)
		void allowAll();
		void blockAll();
		void allow(std::string id);
		void block(std::string id);
		void blockAllBut(std::string id);
		void allowAllBut(std::string id);

	protected:
		std::shared_ptr<Impl> m_impl;
};


} // Events
} // FW


#endif /* FWEVENTHANDLER_H_ */
