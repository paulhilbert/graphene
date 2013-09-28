inline Handle::Handle(std::string id, EventHandler::Ptr eventHandler) : m_id(id), m_eventHandler(eventHandler) {
}

inline Handle::~Handle() {
	disconnect();
}

template <class Sig>
inline void Handle::connect(EventType type, std::function<Sig> receiver) {
	m_eventHandler->registerReceiver<Sig>(type, m_id, receiver);
}

inline void Handle::disconnect(EventType type) {
	if (type == "__all__") m_eventHandler->unregisterReceiver(m_id);
	else m_eventHandler->unregisterReceiver(type, m_id);
}
