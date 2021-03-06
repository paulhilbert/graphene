/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5GLWIDGET_H_
#define QT5GLWIDGET_H_

#include <include/common.h>
#include <include/ogl.h>
#include <QtOpenGL/QGLWidget>
#include <QtGui/QMouseEvent>

#include <FW/Events/EventsEventHandler.h>
using FW::Events::EventHandler;
#include <GUI/Property/PropColor.h>
using GUI::Property::Color;


namespace GUI {

class Qt5GLWidget : public QGLWidget {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5GLWidget> Ptr;
		typedef std::weak_ptr<Qt5GLWidget>   WPtr;
		typedef enum {Left, Right, Middle}   MouseButton;

	public:
		Qt5GLWidget(EventHandler::Ptr eventHandler, int mouseTolerance);
		virtual ~Qt5GLWidget();

		void registerCallbacks();
		void setRenderCallback(std::function<void ()> renderCallback);

		Eigen::Vector2i size() const;

	protected:
		void initializeGL();
		void resizeGL(int w, int h);
		void paintGL();
		void mouseMoveEvent(QMouseEvent* event);
		void mousePressEvent(QMouseEvent* event);
		void mouseReleaseEvent(QMouseEvent* event);
		void wheelEvent(QWheelEvent* event);
		bool eventFilter(QObject*, QEvent* event);

		void mousePress(MouseButton btn, int x, int y);
		void mouseRelease(MouseButton btn, int x, int y);
		void mouseClick(MouseButton btn, int x, int y);
		void mouseMove(int dx, int dy, int x, int y);
		void mouseDrag(MouseButton btn, int dx, int dy, int x, int y);
		void mouseDragStart(MouseButton btn, int x, int y);
		void mouseDragStop(MouseButton btn, int x, int y);
		void mouseScroll(int delta);

	protected:
		EventHandler::Ptr       m_eventHandler;
		std::function<void ()>  m_renderCallback;
		int                     m_width;
		int                     m_height;
		// mouse tracking
		int                     m_mouseTolerance;
		int                     m_x;
		int                     m_y;
		int                     m_startX;
		int                     m_startY;
		bool                    m_mouseDragged;
		bool                    m_leftDown;
		bool                    m_middleDown;
		bool                    m_rightDown;
		int                     m_scrollPos;
};


} // GUI

#endif /* QT5GLWIDGET_H_ */
