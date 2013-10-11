#include "Qt5Backend.h"

#include <GUI/Property/Button.h>
#include <GUI/Property/Color.h>
#include <GUI/Property/PropGroup.h>
using namespace GUI::Property;

#include "Mode/Qt5Handle.h"
#include "Qt5Status.h"
#include "Qt5Log.h"

#include <QtCore/QTimer>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QFileDialog>


namespace GUI {

Qt5Backend::Qt5Backend() : Backend(), m_glWidget(nullptr) {
}

Qt5Backend::~Qt5Backend() {
	delete m_app;
}

void Qt5Backend::init(int argc, char* argv[], FW::Events::EventHandler::Ptr eventHandler, bool verbose) {
	m_app = new QApplication(argc, argv);
	m_wnd = new QMainWindow();

	// setup log window
	m_logDialog = new Qt5LogDialog("Log");
	m_logDialog->setAllowedAreas(Qt::BottomDockWidgetArea);
	m_wnd->addDockWidget(Qt::BottomDockWidgetArea, m_logDialog);
	m_logDialog->hide();
	m_log = Qt5Log::Ptr(new Qt5Log(m_logDialog, verbose));

	// setup settings area
	m_dock = new QDockWidget();
	m_dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
	m_dock->setMinimumSize(QSize(400, 400));
	m_settings = Qt5Settings::Ptr(new Qt5Settings(m_log));
	m_dock->setWidget(m_settings->widget());
	QObject::connect(m_settings->widget(), SIGNAL(tabCloseRequested(int)), this, SLOT(tabClose(int)));
	m_wnd->addDockWidget(Qt::RightDockWidgetArea, m_dock);
	initMainSettings();

	// setup add visualizer dialog
	m_addVisDialog = new Qt5AddVisDialog("Add Visualizer");

	// toolbar and status bar
	addToolbar();
	m_wnd->statusBar();

	// init opengl
	m_glWidget = new Qt5GLWidget(eventHandler, 1);
	m_wnd->setCentralWidget(m_glWidget);
	QCoreApplication::instance()->installEventFilter(m_glWidget);
}

int Qt5Backend::run(int fps) {
	QTimer *timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(update()));
	int timeout = 1000 / fps;
	if (!timeout) timeout = 1;
	timer->start(timeout);
	m_wnd->show();
	return m_app->exec();
}

void Qt5Backend::exitApplication() {
	if (m_onExit) m_onExit();
	m_app->quit();
}

FactoryHandle::Ptr Qt5Backend::addFactory(std::string name) {
	Container::Ptr properties = m_addVisDialog->addFactory(name);
	return FactoryHandle::Ptr(new FactoryHandle(properties));
}

VisualizerHandle::Ptr Qt5Backend::addVisualizer(std::string name) {
	Container::Ptr properties = m_settings->add(name);
	if (!properties) {
		m_log->error("Could not add visualizer");
		return VisualizerHandle::Ptr();
	}
	Mode::Qt5Handle::Ptr modes(new Mode::Qt5Handle(m_tbar, m_log));
	Qt5Status::Ptr status(new Qt5Status(m_wnd));
	return VisualizerHandle::Ptr(new VisualizerHandle(properties, std::dynamic_pointer_cast<Mode::Handle>(modes), m_log, status));
}

Log::Ptr Qt5Backend::getLog() {
	return m_log;
}

Status::Ptr Qt5Backend::getStatus() {
	return Qt5Status::Ptr(new Qt5Status(m_wnd));
}

IO::AbstractProgressBarPool::Ptr Qt5Backend::getProgressBarPool() {
	return m_logDialog->progressBarPool();
}

void Qt5Backend::setWindowTitle(const char* title) {
	if (m_wnd) m_wnd->setWindowTitle(title);
}

void Qt5Backend::setWindowSize(int width, int height) {
	if (m_wnd) m_wnd->resize(width, height);
}

Eigen::Vector2i Qt5Backend::getGLSize() {
	return m_glWidget->size();
}

Eigen::Vector4f Qt5Backend::getBackgroundColor() const {
	std::vector<std::string> p(2);
	p[0] = "groupRendering";
	p[1] = "background";
	return m_mainSettings->get<Color>(p)->value();
}

std::vector<std::string> Qt5Backend::getActiveVisualizerNames() {
	return m_settings->getActiveTabs();
}

void Qt5Backend::setRenderCallback(std::function<void ()> func) {
	m_glWidget->setRenderCallback(func);
}

void Qt5Backend::setExitCallback(std::function<void ()> func) {
	m_onExit = func;
}

void Qt5Backend::addToolbar() {
	m_tbar = m_wnd->addToolBar("Application");
	m_tbar->setFloatable(false);
	m_tbar->setMovable(false);
	m_tbar->show();

	// actions
	auto* addVisAction = new QAction(QIcon("Icons/add.png"), "Add Visualizer", m_wnd);
	auto* exitAction = new QAction(QIcon("Icons/exit.png"), "Exit", m_wnd);
	QObject::connect(addVisAction, SIGNAL(triggered()), this, SLOT(onAddVis()));
	QObject::connect(exitAction, SIGNAL(triggered()), this, SLOT(onExit()));
	m_tbar->addAction(addVisAction);
	m_tbar->addAction(exitAction);
	m_tbar->addSeparator();

	auto* showSettingsButton = new QToolButton(m_wnd);
	showSettingsButton->setIcon(QIcon("Icons/settings.png"));
	showSettingsButton->setText("Show Settings");
	showSettingsButton->setCheckable(true);
	showSettingsButton->setChecked(true);
	QObject::connect(showSettingsButton, SIGNAL(toggled(bool)), this, SLOT(onShowSettings(bool)));
	QObject::connect(m_dock, SIGNAL(visibilityChanged(bool)), showSettingsButton, SLOT(setChecked(bool)));
	m_tbar->addWidget(showSettingsButton);

	auto* showLogButton = new QToolButton(m_wnd);
	showLogButton->setIcon(QIcon("Icons/log.png"));
	showLogButton->setText("Show Log");
	showLogButton->setCheckable(true);
	QObject::connect(showLogButton, SIGNAL(toggled(bool)), this, SLOT(onShowLog(bool)));
	QObject::connect(m_logDialog, SIGNAL(visibilityChanged(bool)), showLogButton, SLOT(setChecked(bool)));
	m_tbar->addWidget(showLogButton);
	auto* clearLogAction = new QAction(QIcon("Icons/clearLog.png"), "Clear Log", m_wnd);
	QObject::connect(clearLogAction, SIGNAL(triggered()), this, SLOT(onClearLog()));
	m_tbar->addAction(clearLogAction);

#ifdef ENABLE_SCREENCAST
	m_tbar->addSeparator();
	m_recording = false;
	m_startScreencastAction = new QAction(QIcon("Icons/record.png"), "Record Screencast", m_wnd);
	m_stopScreencastAction = new QAction(QIcon("Icons/stop.png"), "Record Screencast", m_wnd);
	m_stopScreencastAction->setEnabled(false);
	QObject::connect(m_startScreencastAction, SIGNAL(triggered()), this, SLOT(onRecord()));
	QObject::connect(m_stopScreencastAction, SIGNAL(triggered()), this, SLOT(onStop()));
	m_tbar->addAction(m_startScreencastAction);
	m_tbar->addAction(m_stopScreencastAction);
#endif // ENABLE_SCREENCAST
}

void Qt5Backend::initMainSettings() {
	m_mainSettings = m_settings->add("Global", false);
	if (!m_mainSettings) {
		m_log->error("Could not add global settings page");
		return;
	}
	auto groupNav = m_mainSettings->add<Group>("Navigation", "groupNavigation");
	auto ccChoice = groupNav->add<Choice>("Camera Control:", "camControl");
	ccChoice->add("orbit", "Orbit Control");
	ccChoice->add("fly", "Fly Control");
	ccChoice->setCallback([&] (std::string control) { if (m_onSetCamControl) m_onSetCamControl(control); });
	auto groupRender = m_mainSettings->add<Group>("Rendering", "groupRendering");
	groupRender->add<Color>("Background: ", "background")->setValue(Eigen::Vector4f(0.f, 0.f, 0.f, 1.f));
	auto projection = groupRender->add<Choice>("Projection:");
	projection->add("perspective", "Perspective");
	projection->add("ortho", "Orthographic");
	projection->setCallback([&] (std::string proj) { if (m_onSetOrtho) m_onSetOrtho(proj == "ortho"); });
}

void Qt5Backend::update() {
	m_glWidget->updateGL();
}

void Qt5Backend::onAddVis() {
	if (m_addVisDialog->exec() == QDialog::Accepted) {
		if (m_onAddVis) m_onAddVis(m_addVisDialog->getActiveFactory(), m_addVisDialog->getActiveVisName());
	}
}

void Qt5Backend::onExit() {
	if (m_onExit) m_onExit();
	m_app->exit();
}

void Qt5Backend::onClearLog() {
	m_log->clear();
}


void Qt5Backend::tabClose(int index) {
	if (!index) return;
	if (m_onDelVis) m_onDelVis(m_settings->remove(index));
}

void Qt5Backend::onShowLog(bool checked) {
	m_logDialog->setVisible(checked);
}

void Qt5Backend::onShowSettings(bool checked) {
	m_dock->setVisible(checked);
}

void Qt5Backend::onRecord() {
#ifdef ENABLE_SCREENCAST
	if (m_recording) {
		m_startScreencastAction->setIcon(QIcon("Icons/record.png"));
		if (m_onPauseScreencast) m_onPauseScreencast();
	} else {
		if (!m_stopScreencastAction->isEnabled()) {
			// we are recording to a new file (not just pausing)
			QString filter = tr("MP4 Files (*.mp4)");
			QString file = QFileDialog::getSaveFileName(nullptr, "Save...", QString(), filter, nullptr, 0);
			if (file == "") return;
			m_stopScreencastAction->setEnabled(true);
			if (m_onStartScreencast) m_onStartScreencast(fs::path(file.toStdString()));
		} else {
			if (m_onResumeScreencast) m_onResumeScreencast();
		}
		m_startScreencastAction->setIcon(QIcon("Icons/pause.png"));
	}
	m_recording = !m_recording;


	/*
	QString filter;
	if (!m_extensions.size()) {
		filter = tr("All Files (*.*)");
	} else {
		filter = tr("Valid Files (");
		for (unsigned int i=0; i<m_extensions.size(); ++i) {
			if (i) filter += " ";
			filter += "*."+QString::fromStdString(m_extensions[i]);
		}
		filter += tr(")");
	}
	*/
#endif // ENABLE_SCREENCAST
}

void Qt5Backend::onStop() {
#ifdef ENABLE_SCREENCAST
	m_recording = false;
	m_startScreencastAction->setIcon(QIcon("Icons/record.png"));
	m_stopScreencastAction->setEnabled(false);
	if (m_onStopScreencast) m_onStopScreencast();
#endif // ENABLE_SCREENCAST
}


} // GUI
