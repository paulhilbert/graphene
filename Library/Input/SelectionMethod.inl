/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


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
