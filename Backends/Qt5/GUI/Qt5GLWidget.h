#ifndef QT5GLWIDGET_H_
#define QT5GLWIDGET_H_

#include <Eigen/Dense>
#include <GL/glew.h>
#include <QtOpenGL/QGLWidget>
#include <QtGui/QMouseEvent>

#include <FW/Events/EventHandler.h>
using FW::Events::EventHandler;
#include <GUI/Property/Color.h>
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
