/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5BACKEND_H_
#define QT5BACKEND_H_

#include <include/common.h>

#include <QtCore/QObject>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QTabWidget>
#include "Qt5GLWidget.h"

#include <GUI/GUIBackend.h>

#include "Qt5Settings.h"
#include "Qt5AddVisDialog.h"
#include "Qt5LogDialog.h"

namespace GUI {


class Qt5Backend : public QObject, public Backend {
	Q_OBJECT

	public:
		Qt5Backend();
		virtual ~Qt5Backend();

		void init(int argc, char* argv[], FW::Events::EventHandler::Ptr eventHandler, const WindowParams& params, bool singleMode, bool verbose);
		int  run(int fps);

		FactoryHandle::Ptr addFactory(std::string name);
		VisualizerHandle::Ptr addVisualizer(std::string name);

		Log::Ptr getLog();
		Status::Ptr getStatus();
		ProgressBarPool::Ptr getProgressBarPool();
		Container::Ptr getMainSettings();

		void setWindowTitle(std::string title);
		void setWindowSize(int width, int height);
		void setStylesheet(std::string stylesheet);
		Eigen::Vector2i getGLSize();

		Eigen::Vector4f getBackgroundColor() const;
		std::vector<std::string> getActiveVisualizerNames();

		void setRenderCallback(std::function<void ()> func);

	protected:
		bool initSingleVisualizer();
		void addToolbar();

	public slots:
		void update();
		bool onAddVis();
		void onExit();
		void onClearLog();
		void tabClose(int idx);
		void onShowLog(bool checked);
		void onShowSettings(bool checked);
		void onRecord();
		void onStop();

	protected:
		bool                 m_singleMode;
		QApplication*        m_app;
		QMainWindow*         m_wnd;
		QToolBar*            m_tbar;
		Qt5GLWidget*         m_glWidget;
		QDockWidget*         m_dock;
		Qt5Settings::Ptr     m_settings;
		Container::Ptr       m_mainSettings;
		Qt5AddVisDialog*     m_addVisDialog;
		Qt5LogDialog*        m_logDialog;
		Log::Ptr             m_log;
		std::function<void (void)> m_onExit;

#ifdef ENABLE_SCREENCAST
		QAction*          m_startScreencastAction;
		QAction*          m_stopScreencastAction;
		bool              m_recording;
#endif // ENABLE_SCREENCAST
};


} // GUI

#endif /* QT5BACKEND_H_ */
