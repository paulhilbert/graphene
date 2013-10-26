/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


inline Handle::Handle(std::string id, EventHandler::Ptr eventHandler) : m_id(id), m_eventHandler(eventHandler) {
}

inline Handle::~Handle() {
	disconnect();
}

template <class Sig>
inline void Handle::connect(std::string type, std::function<Sig> receiver) {
	m_eventHandler->registerReceiver<Sig>(type, m_id, receiver);
}

inline void Handle::disconnect(std::string type) {
	if (type == "__all__") m_eventHandler->unregisterReceiver(m_id);
	else m_eventHandler->unregisterReceiver(type, m_id);
}
