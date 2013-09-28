#include "Backend.h"
#include <Testing/asserts.h>

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
