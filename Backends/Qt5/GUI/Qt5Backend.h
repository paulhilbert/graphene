#ifndef QT5BACKEND_H_
#define QT5BACKEND_H_

#include <memory>

#include <QtCore/QObject>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QTabWidget>
#include "Qt5GLWidget.h"

#include <GUI/Backend.h>

#include "Qt5Settings.h"
#include "Qt5AddVisDialog.h"
#include "Qt5LogDialog.h"

namespace GUI {


class Qt5Backend : public QObject, public Backend {
	Q_OBJECT

	public:
		Qt5Backend();
		virtual ~Qt5Backend();

		void init(int argc, char* argv[], FW::Events::EventHandler::Ptr eventHandler, bool verbose);
		int  run(int fps);
		void exitApplication();

		FactoryHandle::Ptr addFactory(std::string name);
		VisualizerHandle::Ptr addVisualizer(std::string name);

		Log::Ptr getLog();
		Status::Ptr getStatus();
		IO::AbstractProgressBarPool::Ptr getProgressBarPool();

		void setWindowTitle(const char* title);
		void setWindowSize(int width, int height);
		Eigen::Vector2i getGLSize();

		Eigen::Vector4f getBackgroundColor() const;
		std::vector<std::string> getActiveVisualizerNames();

		void setRenderCallback(std::function<void ()> func);
		void setExitCallback(std::function<void ()> func);

	protected:
		void addToolbar();
		void initMainSettings();

	public slots:
		void update();
		void onAddVis();
		void onExit();
		void onClearLog();
		void tabClose(int idx);
		void onShowLog(bool checked);
		void onShowSettings(bool checked);
		void onRecord();
		void onStop();

	protected:
		QApplication*     m_app;
		QMainWindow*      m_wnd;
		QToolBar*         m_tbar;
		Qt5GLWidget*      m_glWidget;
		QDockWidget*      m_dock;
		Qt5Settings::Ptr  m_settings;
		Container::Ptr    m_mainSettings;
		Qt5AddVisDialog*  m_addVisDialog;
		Qt5LogDialog*     m_logDialog;
		Log::Ptr          m_log;
		std::function<void (void)> m_onExit;

#ifdef ENABLE_SCREENCAST
		QAction*          m_startScreencastAction;
		QAction*          m_stopScreencastAction;
		bool              m_recording;
#endif // ENABLE_SCREENCAST
};


} // GUI

#endif /* QT5BACKEND_H_ */
