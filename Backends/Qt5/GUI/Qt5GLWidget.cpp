#include "Qt5GLWidget.h"

#include <FW/Events/Keys.h>

namespace GUI {

Qt5GLWidget::Qt5GLWidget(EventHandler::Ptr eventHandler, int mouseTolerance) : 
	QGLWidget(),
	m_eventHandler(eventHandler),
	m_mouseTolerance(mouseTolerance),
	m_mouseDragged(false),
	m_leftDown(false),
	m_middleDown(false),
	m_rightDown(false),
	m_scrollPos(0) {
	setMouseTracking(true);
}

Qt5GLWidget::~Qt5GLWidget() {
}

void Qt5GLWidget::registerCallbacks() {
}

void Qt5GLWidget::setRenderCallback(std::function<void ()> renderCallback) {
	m_renderCallback = std::move(renderCallback);
}

Eigen::Vector2i Qt5GLWidget::size() const {
	return Eigen::Vector2i(m_width, m_height);
}

void Qt5GLWidget::initializeGL() {
	glewInit();
}

void Qt5GLWidget::resizeGL(int w, int h) {
	FW::Events::Signal signal("WINDOW_RESIZE");
	signal.addParam(w);
	signal.addParam(h);
	m_eventHandler->notify(signal);
	glViewport(0, 0, (GLint)w, (GLint)h);
	m_width = w;
	m_height = h;
}

void Qt5GLWidget::paintGL() {
//	glViewport(0, 0, (GLint)m_width, (GLint)m_height);
	if (m_renderCallback) m_renderCallback();
}

void Qt5GLWidget::mouseMoveEvent(QMouseEvent* event) {
	int x = event->x();
	int y = m_height - event->y() + 1;

	int dx = x - m_x;
	int dy = m_y - y;
	m_x = x;
	m_y = y;

	if ((m_leftDown || m_rightDown || m_middleDown)
	&& (abs(x - m_startX) > m_mouseTolerance || abs(y - m_startY) > m_mouseTolerance)) {
		if (!m_mouseDragged) {
			// we are just starting to drag
			if (m_leftDown) mouseDragStart(Left, m_startX, m_startY);
			if (m_rightDown) mouseDragStart(Right, m_startX, m_startY);
			if (m_middleDown) mouseDragStart(Middle, m_startX, m_startY);
		}
		m_mouseDragged = true;

		if (m_leftDown) mouseDrag(Left, dx, dy, x, y);
		if (m_rightDown) mouseDrag(Right, dx, dy, x, y);
		if (m_middleDown) mouseDrag(Middle, dx, dy, x, y);

	} else {
		mouseMove(dx, dy, x, y);
	}
}

void Qt5GLWidget::mousePressEvent(QMouseEvent* event) {
	m_x = event->x();
	m_y = m_height - event->y() - 1;

	m_startX = m_x;
	m_startY = m_y;

	switch (event->button()) {
		case Qt::LeftButton:  m_leftDown = true; break;
		case Qt::RightButton: m_rightDown = true; break;
		case Qt::MidButton:   m_middleDown = true; break;
		default: break;
	}
}

void Qt5GLWidget::mouseReleaseEvent(QMouseEvent* event) {
	int x = event->x();
	int y = m_height - event->y() - 1;
	switch (event->button()) {
		case Qt::LeftButton:
			if (!m_mouseDragged) mouseClick(Left, m_startX, m_startY);
			else mouseDragStop(Left, x, y);
			m_leftDown = false;
			break;
		case Qt::RightButton:
			if (!m_mouseDragged) mouseClick(Right, m_startX, m_startY);
			else mouseDragStop(Left, x, y);
			m_rightDown = false;
			break;
		case Qt::MiddleButton:
			if (!m_mouseDragged) mouseClick(Middle, m_startX, m_startY);
			else mouseDragStop(Left, x, y);
			m_middleDown = false;
			break;
		default: break;
	}
	m_mouseDragged = false;
}

void Qt5GLWidget::wheelEvent(QWheelEvent* event) {
	QPoint numDegrees = event->angleDelta() / 120;
	mouseScroll(numDegrees.y());
}

bool Qt5GLWidget::eventFilter(QObject*, QEvent* event) {
	if (event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease) {
		QKeyEvent* keyEvent = (QKeyEvent*)event;
		std::string suffix = event->type() == QEvent::KeyPress ? "_PRESS" : "_RELEASE";
		if (keyEvent->key() == Qt::Key_Shift || keyEvent->key() == Qt::Key_Control || keyEvent->key() == Qt::Key_Alt || keyEvent->key() == Qt::Key_AltGr) {
			FW::Events::Keys::Modifier mod;
			switch (keyEvent->key()) {
				case Qt::Key_Shift:   mod = FW::Events::Keys::SHIFT; break;
				case Qt::Key_Control: mod = FW::Events::Keys::CTRL; break;
				case Qt::Key_Alt:     mod = FW::Events::Keys::ALT; break;
				default:              mod = FW::Events::Keys::ALTGR; break;
			}
			FW::Events::Signal signal("MODIFIER"+suffix);
			signal.addParam(mod);
			m_eventHandler->notify(signal);
		} else if (keyEvent->key() >= Qt::Key_Escape && keyEvent->key() <= Qt::Key_F12) {
			FW::Events::Keys::Special special;
			switch (keyEvent->key()) {
				case Qt::Key_Escape: special = FW::Events::Keys::ESCAPE; break;
				case Qt::Key_Tab: special = FW::Events::Keys::TAB; break;
				case Qt::Key_Backtab: special = FW::Events::Keys::BACKTAB; break;
				case Qt::Key_Backspace: special = FW::Events::Keys::BACKSPACE; break;
				case Qt::Key_Return: special = FW::Events::Keys::RETURN; break;
				case Qt::Key_Enter: special = FW::Events::Keys::ENTER; break;
				case Qt::Key_Insert: special = FW::Events::Keys::INSERT; break;
				case Qt::Key_Delete: special = FW::Events::Keys::DELETE; break;
				case Qt::Key_Pause: special = FW::Events::Keys::PAUSE; break;
				case Qt::Key_Print: special = FW::Events::Keys::PRINT; break;
				case Qt::Key_SysReq: special = FW::Events::Keys::SYSREQ; break;
				case Qt::Key_Clear: special = FW::Events::Keys::CLEAR; break;
				case Qt::Key_Home: special = FW::Events::Keys::HOME; break;
				case Qt::Key_End: special = FW::Events::Keys::END; break;
				case Qt::Key_Left: special = FW::Events::Keys::LEFT; break;
				case Qt::Key_Up: special = FW::Events::Keys::UP; break;
				case Qt::Key_Right: special = FW::Events::Keys::RIGHT; break;
				case Qt::Key_Down: special = FW::Events::Keys::DOWN; break;
				case Qt::Key_PageUp: special = FW::Events::Keys::PAGEUP; break;
				case Qt::Key_PageDown: special = FW::Events::Keys::PAGEDOWN; break;
				case Qt::Key_Meta: special = FW::Events::Keys::META; break;
				case Qt::Key_CapsLock: special = FW::Events::Keys::CAPSLOCK; break;
				case Qt::Key_NumLock: special = FW::Events::Keys::NUMLOCK; break;
				case Qt::Key_ScrollLock: special = FW::Events::Keys::SCROLLLOCK; break;
				case Qt::Key_F1: special = FW::Events::Keys::F1; break;
				case Qt::Key_F2: special = FW::Events::Keys::F2; break;
				case Qt::Key_F3: special = FW::Events::Keys::F3; break;
				case Qt::Key_F4: special = FW::Events::Keys::F4; break;
				case Qt::Key_F5: special = FW::Events::Keys::F5; break;
				case Qt::Key_F6: special = FW::Events::Keys::F6; break;
				case Qt::Key_F7: special = FW::Events::Keys::F7; break;
				case Qt::Key_F8: special = FW::Events::Keys::F8; break;
				case Qt::Key_F9: special = FW::Events::Keys::F9; break;
				case Qt::Key_F10: special = FW::Events::Keys::F10; break;
				case Qt::Key_F11: special = FW::Events::Keys::F11; break;
				default: special = FW::Events::Keys::F12; break;
			}
			FW::Events::Signal signal("SPECIAL_KEY"+suffix);
			signal.addParam(special);
			m_eventHandler->notify(signal);
		} else if (keyEvent->text() != "") {
			FW::Events::Signal signal("CHAR_KEY"+suffix);
			signal.addParam(keyEvent->text().toStdString());
			m_eventHandler->notify(signal);
		}
	}
	return false;
}

void Qt5GLWidget::mouseClick(MouseButton btn, int x, int y) {
	if (!m_eventHandler) return;

	FW::Events::EventType type;
	switch (btn) {
		case Left: type = "LEFT_CLICK"; break;
		case Right: type = "RIGHT_CLICK"; break;
		default: type = "MIDDLE_CLICK";
	}

	FW::Events::Signal signal(type);
	signal.addParam(x);
	signal.addParam(y);
	m_eventHandler->notify(signal);
}

void Qt5GLWidget::mouseMove(int dx, int dy, int x, int y) {
	if (!m_eventHandler) return;

	FW::Events::Signal signal("MOVE");
	signal.addParam(dx);
	signal.addParam(dy);
	signal.addParam(x);
	signal.addParam(y);
	m_eventHandler->notify(signal);
}

void Qt5GLWidget::mouseDrag(MouseButton btn, int dx, int dy, int x, int y) {
	if (!m_eventHandler) return;

	FW::Events::EventType type;
	switch (btn) {
		case Left: type = "LEFT_DRAG"; break;
		case Right: type = "RIGHT_DRAG"; break;
		default: type = "MIDDLE_DRAG";
	}

	FW::Events::Signal signal(type);
	signal.addParam(dx);
	signal.addParam(dy);
	signal.addParam(x);
	signal.addParam(y);
	m_eventHandler->notify(signal);
}

void Qt5GLWidget::mouseDragStart(MouseButton btn, int x, int y) {
	if (!m_eventHandler) return;

	FW::Events::EventType type;
	switch (btn) {
		case Left: type = "LEFT_DRAG_START"; break;
		case Right: type = "RIGHT_DRAG_START"; break;
		default: type = "MIDDLE_DRAG_START";
	}

	FW::Events::Signal signal(type);
	signal.addParam(x);
	signal.addParam(y);
	m_eventHandler->notify(signal);
}

void Qt5GLWidget::mouseDragStop(MouseButton btn, int x, int y) {
	if (!m_eventHandler) return;

	FW::Events::EventType type;
	switch (btn) {
		case Left: type = "LEFT_DRAG_END"; break;
		case Right: type = "RIGHT_DRAG_END"; break;
		default: type = "MIDDLE_DRAG_END";
	}

	FW::Events::Signal signal(type);
	signal.addParam(x);
	signal.addParam(y);
	m_eventHandler->notify(signal);
}

void Qt5GLWidget::mouseScroll(int delta) {
	if (!m_eventHandler) return;

	FW::Events::Signal signal("SCROLL");
	signal.addParam(delta);
	m_eventHandler->notify(signal);
}

} // GUI
