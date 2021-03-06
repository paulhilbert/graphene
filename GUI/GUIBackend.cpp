/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "GUIBackend.h"

namespace GUI {

Backend::Backend() {
}

Backend::~Backend() {
}

void Backend::setAddVisCallback(const std::function<void (std::string, std::string)>& onAddVis) {
	m_onAddVis = onAddVis;
}

void Backend::setDelVisCallback(const std::function<void (std::string)>& onDelVis) {
	m_onDelVis = onDelVis;
}

void Backend::setExitCallback(std::function<void ()> func) {
	m_onExit = func;
}

void Backend::setCamControlCallback(std::function<void (std::string)> onSetCamControl) {
	m_onSetCamControl = std::move(onSetCamControl);
}

void Backend::setOrthoCallback(std::function<void (bool)> onSetOrtho) {
	m_onSetOrtho = onSetOrtho;
}

#ifdef ENABLE_SCREENCAST
void Backend::setScreencastStartCallback(const std::function<void (fs::path)>& func) {
	m_onStartScreencast = std::move(func);
}

void Backend::setScreencastPauseCallback(const std::function<void (void)>& func) {
	m_onPauseScreencast = std::move(func);
}

void Backend::setScreencastResumeCallback(const std::function<void (void)>& func) {
	m_onResumeScreencast = std::move(func);
}

void Backend::setScreencastStopCallback(const std::function<void (void)>& func) {
	m_onStopScreencast = std::move(func);
}
#endif // ENABLE_SCREENCAST


} // GUI
