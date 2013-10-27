/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "EventsEventHandler.h"

#include <FW/Events/EventsKeys.h>

namespace FW {
namespace Events {


class Signal::Impl {
	public:
		Impl(EventType type);
		void addParam(boost::any param);
		const vector<boost::any>& getParams() const;
		EventType getType() const;

		EventType m_type;
		Params    m_params;
};

class EventHandler::Impl {
	public:
		typedef std::shared_ptr<BaseCommand>  CmdPtr;
		struct Receiver {
			std::string id;
			bool blocked;
			CmdPtr receiver;
		};
		typedef vector<Receiver> Receivers;
		typedef std::map<EventType, Receivers> ReceiverMap;

		Impl();

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

		ReceiverMap  m_receivers;
		Modifier::Ptr m_modifier;
};


/// SIGNAL ///

Signal::Signal(EventType type) {
	m_impl = std::shared_ptr<Impl>( new Impl(type) );
}

void Signal::addParam(Param param) {
	m_impl->addParam(param);
}

const Signal::Params& Signal::getParams() const {
	return m_impl->getParams();
}

EventType Signal::getType() const {
	return m_impl->getType();
}


/// SIGNAL IMPL ///


Signal::Impl::Impl(EventType type) : m_type(type) {
}

void Signal::Impl::addParam(Param param) {
	m_params.push_back(param);
}

const Signal::Params& Signal::Impl::getParams() const {
	return m_params;
}

EventType Signal::Impl::getType() const {
	return m_type;
}


/// EVENT HANDLER ///


EventHandler::EventHandler() {
	m_impl = std::shared_ptr<Impl>(new Impl());
}

template <class Sig>
void EventHandler::registerReceiver(EventType eventType, std::string id, std::function<Sig> receiver) {
	m_impl->registerReceiver<Sig>(eventType, id, receiver);
}

//template <>
//void EventHandler::registerReceiver(EventType eventType, std::string id, std::function<void (void)> receiver) {
//	m_impl->registerReceiver<void (void)>(eventType, id, receiver);
//}
//
//template <>
//void EventHandler::registerReceiver(EventType eventType, std::string id, std::function<void (int, int)> receiver) {
//	m_impl->registerReceiver<void (int, int)>(eventType, id, receiver);
//}
//
//template <>
//void EventHandler::registerReceiver(EventType eventType, std::string id, std::function<void (int, int, int, int)> receiver) {
//	m_impl->registerReceiver<void (int, int, int, int)>(eventType, id, receiver);
//}

void EventHandler::unregisterReceiver(EventType eventType, std::string id) {
	m_impl->unregisterReceiver(eventType, id);
}

void EventHandler::unregisterReceiver(std::string id) {
	m_impl->unregisterReceiver(id);
}

void EventHandler::notify(Signal signal) {
	m_impl->notify(signal);
}

Modifier::Ptr EventHandler::modifier() {
	return m_impl->modifier();
}

void EventHandler::allowAll() {
	m_impl->allowAll();
}

void EventHandler::blockAll() {
	m_impl->blockAll();
}

void EventHandler::allow(std::string id) {
	m_impl->allow(id);
}

void EventHandler::block(std::string id) {
	m_impl->block(id);
}

void EventHandler::blockAllBut(std::string id) {
	blockAll();
	allow(id);
}

void EventHandler::allowAllBut(std::string id) {
	allowAll();
	block(id);
}


/// EVENT HANDLER IMPL ///


EventHandler::Impl::Impl() : m_modifier(new Modifier()) {
}

template <class Sig>
void EventHandler::Impl::registerReceiver(EventType eventType, std::string id, std::function<Sig> receiver) {
	if (m_receivers.find(eventType) == m_receivers.end()) {
		Receivers newReceivers;
		m_receivers[eventType] = newReceivers;
	}
	Receiver newReceiver = { id, false, CmdPtr(new Command<Sig>(receiver)) };
	m_receivers[eventType].push_back(newReceiver);
}

void EventHandler::Impl::unregisterReceiver(EventType eventType, std::string id) {
	ReceiverMap::iterator findIt = m_receivers.find(eventType);
	if (findIt == m_receivers.end()) return;

	auto it = findIt->second.begin();
	while (it != findIt->second.end()) {
		if ( it->id == id ) {
			it = findIt->second.erase(it);
		} else {
			++it;
		}
	}
	if (!findIt->second.size()) m_receivers.erase(findIt);
}

void EventHandler::Impl::unregisterReceiver(std::string id) {
	std::vector<std::string> toErase;
	for (ReceiverMap::iterator eventIt = m_receivers.begin(); eventIt != m_receivers.end(); ++eventIt) {
		Algorithm::remove(eventIt->second, [&](const Receiver& r) { return r.id == id; });
		/*
		for (Receivers::iterator it = eventIt->second.begin(); it != eventIt->second.end(); ++it) {
			if ( it->id == id ) eventIt->second.erase(it);
		}
		*/

		if (!eventIt->second.size()) toErase.push_back(eventIt->first);
	}
	for (const auto& key : toErase) {
		m_receivers.erase(key);
	}
}

void EventHandler::Impl::notify(Signal signal) {
	EventType type = signal.getType();

	// PRE EVENT
	if (type != "PRE_EVENT" && type != "POST_EVENT") {
		Signal preSignal("PRE_EVENT");
		preSignal.addParam(type);
		notify(preSignal);
	}

	// EVENT
	ReceiverMap::iterator findIt = m_receivers.find(type);
	if (findIt != m_receivers.end() && findIt->second.size()) {
		for (unsigned int i=0; i < findIt->second.size(); ++i) {
			if (findIt->second[i].blocked) continue;
			findIt->second[i].receiver->setParams(signal.getParams());
			(*findIt->second[i].receiver)();
		}
	}

	// POST EVENT
	if (type != "PRE_EVENT" && type != "POST_EVENT") {
		Signal postSignal("POST_EVENT");
		postSignal.addParam(type);
		notify(postSignal);
	}
}

Modifier::Ptr EventHandler::Impl::modifier() {
	return m_modifier;
}

void EventHandler::Impl::allowAll() {
	for (ReceiverMap::iterator eventIt = m_receivers.begin(); eventIt != m_receivers.end(); ++eventIt) {
		for (Receivers::iterator it = eventIt->second.begin(); it != eventIt->second.end(); ++it) {
			it->blocked = false;
		}
	}
}

void EventHandler::Impl::blockAll() {
	for (ReceiverMap::iterator eventIt = m_receivers.begin(); eventIt != m_receivers.end(); ++eventIt) {
		for (Receivers::iterator it = eventIt->second.begin(); it != eventIt->second.end(); ++it) {
			it->blocked = true;
		}
	}
}

void EventHandler::Impl::allow(std::string id) {
	for (ReceiverMap::iterator eventIt = m_receivers.begin(); eventIt != m_receivers.end(); ++eventIt) {
		for (Receivers::iterator it = eventIt->second.begin(); it != eventIt->second.end(); ++it) {
			if ( it->id == id ) it->blocked = false;
		}
	}
}

void EventHandler::Impl::block(std::string id) {
	for (ReceiverMap::iterator eventIt = m_receivers.begin(); eventIt != m_receivers.end(); ++eventIt) {
		for (Receivers::iterator it = eventIt->second.begin(); it != eventIt->second.end(); ++it) {
			if ( it->id == id ) it->blocked = true;
		}
	}
}


template void EventHandler::registerReceiver<void (void)>(EventType eventType, std::string id, std::function<void (void)> receiver);
template void EventHandler::registerReceiver<void (int)>(EventType eventType, std::string id, std::function<void (int)> receiver);
template void EventHandler::registerReceiver<void (int, int)>(EventType eventType, std::string id, std::function<void (int, int)> receiver);
template void EventHandler::registerReceiver<void (int, int, int)>(EventType eventType, std::string id, std::function<void (int, int, int)> receiver);
template void EventHandler::registerReceiver<void (int, int, int, int)>(EventType eventType, std::string id, std::function<void (int, int, int, int)> receiver);
template void EventHandler::registerReceiver<void (std::string)>(EventType eventType, std::string id, std::function<void (std::string)> receiver);
template void EventHandler::registerReceiver<void (FW::Events::Keys::Special)>(EventType eventType, std::string id, std::function<void (FW::Events::Keys::Special)> receiver);
template void EventHandler::registerReceiver<void (FW::Events::Keys::Modifier)>(EventType eventType, std::string id, std::function<void (FW::Events::Keys::Modifier)> receiver);

} // Events
} // FW
