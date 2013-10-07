inline SelectionMethod::SelectionMethod(FW::VisualizerHandle::Ptr handle) : m_handle(handle), m_enabled(false), m_dragging(false) {
}

inline SelectionMethod::~SelectionMethod() {
}

inline void SelectionMethod::enable() {
	m_enabled = true;
}

inline void SelectionMethod::disable() {
	m_enabled = false;
	m_dragging = false;
	if (m_unselect) m_unselect();
}

inline void SelectionMethod::setStartCallback(std::function<void ()> func) {
	m_start = std::move(func);
}

inline void SelectionMethod::setDragCallback(std::function<void ()> func) {
	m_drag = std::move(func);
}

inline void SelectionMethod::setStopCallback(std::function<void ()> func) {
	m_stop = std::move(func);
}

inline void SelectionMethod::setUnselectCallback(std::function<void ()> func) {
	m_unselect = std::move(func);
}
