#include "FWGraphene.h"

#include <iostream>
#include <map>
#include <tuple>
#include <queue>
#include <thread>
#include <mutex>
#include <chrono>

#include <FW/View/Camera.h>
#include <FW/View/OrbitCameraControl.h>
#include <FW/View/FlyCameraControl.h>
#include <FW/View/Transforms.h>
using namespace FW::View;

#include <FW/Visualizer.h>


#ifdef OPENGL_EFFECTS
#include <Library/Buffer/Texture.h>
#include <Library/Buffer/FBO.h>
#include <Library/Shader/ShaderProgram.h>
#include <Library/Buffer/Geometry.h>
#endif // OPENGL_EFFECTS

namespace FW {

#ifdef ENABLE_SCREENCAST
struct ScreencastInfo {
	bool                         recording;
	bool                         paused;
	std::shared_ptr<std::thread> thread;
	std::queue<unsigned char*>   queue;
	std::mutex                   queueMutex;
	int                          recWidth;
	int                          recHeight;
};
#endif // ENABLE_SCREENCAST


struct Graphene::Impl {
		Impl(GUI::Backend::Ptr backend, FW::Events::EventHandler::Ptr eventHandler, bool singleMode);
		virtual ~Impl();

		void initTransforms();
#ifdef OPENGL_EFFECTS
		void initEffects();
		void updateEffects(int width, int height);
#endif // OPENGL_EFFECTS

		int run(int fps);
		void exit();

		void         addFactory(std::string name, Factory::Ptr factory);
		Factory::Ptr getFactory(std::string name);
		bool         hasFactory(std::string name);
		void         addVisualizer(std::string factoryName, std::string visName);
		bool         hasVisualizer(std::string visName);
		void         removeVisualizer(std::string visName);

		void         render();
		void         renderBlur(float ratio);

#ifdef ENABLE_SCREENCAST
		void startScreencast(fs::path outputFile);
		void pauseScreencast();
		void resumeScreencast();
		void stopScreencast();
		void encodeScreencast(fs::path outputFile);
		void printDuration();
#endif // ENABLE_SCREENCAST

		void modifier(Keys::Modifier mod, bool down);

		void setCameraControl(std::string control);
		void setOrtho(bool ortho);


		GUI::Backend::Ptr                         m_backend;
		FW::Events::EventHandler::Ptr             m_eventHandler;

		bool                                      m_singleMode;

		std::map<std::string, Factory::Ptr>       m_factories;
		std::map<std::string, Visualizer::Ptr>    m_visualizer;

		Transforms::Ptr                           m_transforms;
		std::map<std::string, CameraControl::Ptr> m_camControls;
		Camera::Ptr                               m_camera;
		int                                       m_fps;
#ifdef ENABLE_SCREENCAST
		ScreencastInfo                            m_scInfo;
		std::chrono::system_clock::time_point     m_lastUpdate;
		std::chrono::system_clock::duration       m_duration;
		GUI::Status::Ptr                          m_status;
#endif // ENABLE_SCREENCAST
#ifdef OPENGL_EFFECTS
		Shader::ShaderProgram                     m_gaussH;
		Shader::ShaderProgram                     m_gaussV;
		Shader::ShaderProgram                     m_compose;
		std::vector<Buffer::FBO>                  m_fbos;
		Buffer::Geometry                          m_geomQuad;
#endif // OPENGL_EFFECTS
};


/// GRAPHENE ///


Graphene::Graphene(GUI::Backend::Ptr backend, FW::Events::EventHandler::Ptr eventHandler, bool singleMode) {
	m_impl = std::shared_ptr<Impl>(new Impl(backend, eventHandler, singleMode));
}

Graphene::~Graphene() {
}

int Graphene::run(int fps) {
	return m_impl->run(fps);
}

void Graphene::addFactory(std::string name, Factory::Ptr factory) {
	m_impl->addFactory(name, factory);
}

Factory::Ptr Graphene::getFactory(std::string name) {
	return m_impl->getFactory(name);
}


/// GRAPHENE IMPL ///


Graphene::Impl::Impl(GUI::Backend::Ptr backend, FW::Events::EventHandler::Ptr eventHandler, bool singleMode) : m_backend(backend), m_eventHandler(eventHandler), m_singleMode(singleMode) {
	backend->setRenderCallback(std::bind(&Graphene::Impl::render, this));
	backend->setExitCallback(std::bind(&Graphene::Impl::exit, this));
	backend->setAddVisCallback(std::bind(&Graphene::Impl::addVisualizer, this, std::placeholders::_1, std::placeholders::_2));
	backend->setDelVisCallback(std::bind(&Graphene::Impl::removeVisualizer, this, std::placeholders::_1));
	m_eventHandler->registerReceiver<void (Keys::Modifier)>("MODIFIER_PRESS",   "mainapp", std::bind(&Graphene::Impl::modifier, this, std::placeholders::_1, true));
	m_eventHandler->registerReceiver<void (Keys::Modifier)>("MODIFIER_RELEASE", "mainapp", std::bind(&Graphene::Impl::modifier, this, std::placeholders::_1, false));
#ifdef ENABLE_SCREENCAST
	backend->setScreencastStartCallback(std::bind(&Graphene::Impl::startScreencast, this, std::placeholders::_1));
	backend->setScreencastPauseCallback(std::bind(&Graphene::Impl::pauseScreencast, this));
	backend->setScreencastResumeCallback(std::bind(&Graphene::Impl::resumeScreencast, this));
	backend->setScreencastStopCallback(std::bind(&Graphene::Impl::stopScreencast, this));
	m_scInfo.recording = false;
	m_scInfo.paused = false;
	m_scInfo.thread = std::shared_ptr<std::thread>();
#endif // ENABLE_SCREENCAST
	initTransforms();
}

Graphene::Impl::~Impl() {
}

void Graphene::Impl::initTransforms() {
	m_transforms  = View::Transforms::Ptr(new View::Transforms());
	auto glSize = m_backend->getGLSize();
	m_transforms->viewport() = Eigen::Vector4i(0, 0, glSize[0], glSize[1]);
	m_eventHandler->registerReceiver<void (int, int)>("WINDOW_RESIZE", "mainapp", std::function<void (int, int)>([&](int w, int h) {
		m_transforms->viewport()[2] = w;
		m_transforms->viewport()[3] = h;
		m_camera->updateTransforms();
	}));
	m_camControls["orbit"] = OrbitCameraControl::Ptr(new OrbitCameraControl());
	m_camControls["fly"]   = FlyCameraControl::Ptr(new FlyCameraControl());
	m_camera = Camera::Ptr(new Camera(m_camControls["orbit"], m_eventHandler, Transforms::WPtr(m_transforms)));
	// init settings
	auto main = m_backend->getMainSettings();
	auto groupNav = main->add<Section>("Navigation", "groupNavigation");
	auto ccChoice = groupNav->add<Choice>("Camera Control:", "camControl");
	ccChoice->add("orbit", "Orbit Control");
	ccChoice->add("fly", "Fly Control");
	ccChoice->setCallback(std::bind(&Graphene::Impl::setCameraControl, this, std::placeholders::_1));
	auto groupRender = main->add<Section>("Rendering", "groupRendering");
	groupRender->add<Color>("Background: ", "background")->setValue(Eigen::Vector4f(0.f, 0.f, 0.f, 1.f));
	auto projection = groupRender->add<Choice>("Projection:");
	projection->add("perspective", "Perspective");
	projection->add("ortho", "Orthographic");
	projection->setCallback([&] (std::string mode) { setOrtho(mode == "ortho"); });
	if (m_singleMode) {
		groupNav->collapse();
		groupRender->collapse();
	}
}

#ifdef OPENGL_EFFECTS
void Graphene::Impl::initEffects() {
	// properties
	auto main = m_backend->getMainSettings();
	auto group = main->add<Section>("Effects", "groupEffects");
	auto fod = group->add<Group>("Field Of Depth", "groupFOD");
	auto blur = fod->add<Range>("Blur Ratio", "blur");
	blur->setDigits(2);
	blur->setMin(0.f);
	blur->setMax(1.f);
	blur->setValue(0.f);
	auto focalPoint = fod->add<Range>("Focal Point", "focalPoint");
	focalPoint->setDigits(2);
	focalPoint->setMin(0.f);
	focalPoint->setMax(1.f);
	focalPoint->setValue(0.0f);
	auto focalArea = fod->add<Range>("Focal Area", "focalArea");
	focalArea->setDigits(2);
	focalArea->setMin(0.f);
	focalArea->setMax(1.f);
	focalArea->setValue(0.5f);
	if (m_singleMode) group->collapse();

	// events
	m_eventHandler->registerReceiver<void (int, int, int, int)>("LEFT_DRAG", "mainapp", [&] (int dx, int dy, int x, int y) {
		if (! (m_eventHandler->modifier()->ctrl() && m_eventHandler->modifier()->shift()) ) return;
		float newVal = m_backend->getMainSettings()->get<Range>({"groupEffects", "groupFOD", "focalArea"})->value() + (-0.01f * dy);
		if (newVal > 1.f) newVal = 1.f;
		if (newVal < 0.f) newVal = 0.f;
		m_backend->getMainSettings()->get<Range>({"groupEffects", "groupFOD", "focalArea"})->setValue(newVal);
	});
	m_eventHandler->registerReceiver<void (int, int, int, int)>("RIGHT_DRAG", "mainapp", [&] (int dx, int dy, int x, int y) {
		if (! (m_eventHandler->modifier()->ctrl() && m_eventHandler->modifier()->shift()) ) return;
		float newVal = m_backend->getMainSettings()->get<Range>({"groupEffects", "groupFOD", "focalPoint"})->value() + (-0.01f * dy);
		if (newVal > 1.f) newVal = 1.f;
		if (newVal < 0.f) newVal = 0.f;
		m_backend->getMainSettings()->get<Range>({"groupEffects", "groupFOD", "focalPoint"})->setValue(newVal);
	});

	// shaders and geometries
	m_gaussH.addShaders("Library/GLSL/effectsQuad.vert", "Library/GLSL/effectsGaussH.frag");
	m_gaussV.addShaders("Library/GLSL/effectsQuad.vert", "Library/GLSL/effectsGaussV.frag");
	m_compose.addShaders("Library/GLSL/effectsQuad.vert", "Library/GLSL/effectsCompose.frag");
	m_gaussH.link();
	m_gaussV.link();
	m_compose.link();
	m_compose.use();
	m_compose.setUniformVar1i("Tex0", 0);
	m_compose.setUniformVar1i("Tex1", 1);
	m_compose.setUniformVar1i("Tex2", 2);
	std::vector<Vector3f> quadVertices;
	quadVertices.push_back(Vector3f(-1.f, -1.f, 0.f));
	quadVertices.push_back(Vector3f( 1.f, -1.f, 0.f));
	quadVertices.push_back(Vector3f( 1.f,  1.f, 0.f));
	quadVertices.push_back(Vector3f(-1.f,  1.f, 0.f));
	std::vector<GLuint> quadIndices(6);
	quadIndices[0] = 0; quadIndices[1] = 1; quadIndices[2] = 2;
	quadIndices[3] = 0; quadIndices[4] = 2; quadIndices[5] = 3;
	m_geomQuad.init();
	m_geomQuad.setVertices(quadVertices);
	m_geomQuad.setIndices(quadIndices);
	m_geomQuad.upload();
	m_geomQuad.enableVertices();
	m_geomQuad.enableIndices();
	m_geomQuad.bindVertices(m_gaussH, "position");
	m_geomQuad.bindVertices(m_gaussV, "position");
	m_geomQuad.bindVertices(m_compose, "position");
	auto wndSize = m_transforms->viewport().tail(2);
	updateEffects(wndSize[0], wndSize[1]);
	m_eventHandler->registerReceiver<void (int width, int height)>("WINDOW_RESIZE", "mainapp", std::bind(&Graphene::Impl::updateEffects, this, std::placeholders::_1, std::placeholders::_2));
}

void Graphene::Impl::updateEffects(int width, int height) {
	m_fbos.clear();
	m_fbos.resize(3);
	m_fbos[0].SetSize(width, height);
	//m_fbos[0].AttachRender(GL_DEPTH_COMPONENT24);
	m_fbos[0].AttachTexture(GL_RGBA32F_ARB);
	m_fbos[0].AttachTexture(GL_DEPTH_COMPONENT24);
	for (unsigned int i=1; i<3; ++i) {
		m_fbos[i].SetSize(int(width/2), int(height/2));
		m_fbos[i].AttachTexture(GL_RGBA32F_ARB, GL_LINEAR);
		// clamping to preserve post-pro from blur leaking
		m_fbos[i].BindTex();
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	}
	Buffer::FBO::Unbind();
	float weights[9] = {0.0677841f, 0.0954044f, 0.121786f, 0.140999f, 0.148054f, 0.140999f, 0.121786f, 0.0954044f, 0.0677841f};
	float offsetsH[9], offsetsV[9];
	for(int i=0; i<9; i++) {
		offsetsH[i] = (i - 4.0f) / float(width/2.0f);
		offsetsV[i] = (i - 4.0f) / float(height/2.0f);
	}
	m_gaussH.use();
	m_gaussH.setUniformVar1f("weights", 9, weights);
	m_gaussH.setUniformVar1f("offsets", 9, offsetsH);
	m_gaussV.use();
	m_gaussV.setUniformVar1f("weights", 9, weights);
	m_gaussV.setUniformVar1f("offsets", 9, offsetsV);
}
#endif // OPENGL_EFFECTS

int Graphene::Impl::run(int fps) {
	m_fps = fps;
	return m_backend->run(fps);
}

void Graphene::Impl::exit() {
	m_visualizer.clear();
}

void Graphene::Impl::addFactory(std::string name, Factory::Ptr factory) {
	if (hasFactory(name)) {
		m_backend->getLog()->error("Factory already exists");
		return;
	}
	auto handle = m_backend->addFactory(name);
	factory->setGUIHandle(handle);
	factory->init();
	m_factories[name] = factory;
}

Factory::Ptr Graphene::Impl::getFactory(std::string name) {
	if (!hasFactory(name)) {
		m_backend->getLog()->error("Factory does not exist");
		return Factory::Ptr();
	}
	return m_factories[name];
}

bool Graphene::Impl::hasFactory(std::string name) {
	return m_factories.find(name) != m_factories.end();
}

void Graphene::Impl::addVisualizer(std::string factoryName, std::string visName) {
#ifdef OPENGL_EFFECTS
	if (!m_fbos.size()) initEffects();
#endif // OPENGL_EFFECTS
	if (hasVisualizer(visName)) {
		m_backend->getLog()->error("Visualizer already exists");
		return;
	}
	if (!hasFactory(factoryName)) {
		m_backend->getLog()->error("Factory does not exist");
		return;
	}
	auto vis = getFactory(factoryName)->addVisualizer();
	if (!vis) return;
	View::Transforms::WPtr transforms(m_transforms);
	VisualizerHandle::Ptr fwHandle(new VisualizerHandle(visName, transforms, m_eventHandler, m_camera->getPickRay()));
	auto guiHandle = m_backend->addVisualizer(visName);
	if (!guiHandle) return;
	vis->setHandles(fwHandle, guiHandle);
	vis->setProgressBarPool(m_backend->getProgressBarPool());
	vis->init();
	m_visualizer[visName] = vis;
	return;
}

bool Graphene::Impl::hasVisualizer(std::string visName) {
	return m_visualizer.find(visName) != m_visualizer.end();
}

void Graphene::Impl::removeVisualizer(std::string visName) {
	if (!hasVisualizer(visName)) {
		m_backend->getLog()->error("Visualizer does not exist");
		return;
	}
	m_visualizer.erase(m_visualizer.find(visName));
}

void Graphene::Impl::render() {
#ifdef OPENGL_EFFECTS
	float blur = 0.f;
	if (m_visualizer.size()) {
		blur = m_backend->getMainSettings()->get<Range>({"groupEffects", "groupFOD", "blur"})->value();
	}
	if (blur > 0.f) {
		m_fbos[0].BindOutput();
	}
#endif // OPENGL_EFFECTS

	Eigen::Vector4f bg = m_backend->getBackgroundColor();
	glClearColor(bg[0], bg[1], bg[2], bg[3]);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (m_singleMode) {
		for (const auto& vis : m_visualizer) {
			vis.second->waitForTasks();
			vis.second->render();
		}
	} else {
		auto names = m_backend->getActiveVisualizerNames();
		for (const auto& name : names) {
			m_visualizer[name]->waitForTasks();
			m_visualizer[name]->render();
		}
	}

#ifdef OPENGL_EFFECTS
	if (blur > 0.f) {
		renderBlur(blur);
	}
#endif // OPENGL_EFFECTS

#ifdef ENABLE_SCREENCAST
	if (m_scInfo.recording && !m_scInfo.paused) {
		unsigned char* px = new unsigned char[3*m_scInfo.recWidth*m_scInfo.recHeight];
		glReadPixels(0, 0, m_scInfo.recWidth, m_scInfo.recHeight, GL_RGB, GL_UNSIGNED_BYTE, (void*)px);
		m_scInfo.queueMutex.lock();
		m_scInfo.queue.push(px);
		m_scInfo.queueMutex.unlock();
	}
#endif // ENABLE_SCREENCAST
}

void Graphene::Impl::renderBlur(float ratio) {
	auto wndSize = m_transforms->viewport().tail(2);
	auto main = m_backend->getMainSettings();
	float focalPoint = main->get<Range>({"groupEffects", "groupFOD", "focalPoint"})->value();
	float focalArea  = main->get<Range>({"groupEffects", "groupFOD", "focalArea"})->value();
	glDisable(GL_DEPTH_TEST);

	// horizontal gauss
	m_fbos[1].BindOutput();
	m_fbos[0].BindTex();
	m_gaussH.use();
	glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0,0, wndSize[0]/2, wndSize[1]/2); // downsampling
	m_geomQuad.bind();
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, (char *)NULL);

	// vertical gauss
	m_fbos[2].BindOutput();
	m_fbos[1].BindTex();
	m_gaussV.use();
	glClear(GL_COLOR_BUFFER_BIT);
	// geomQuad still bound here
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, (char *)NULL);

	// compose
	Buffer::FBO::Unbind();
	glActiveTexture(GL_TEXTURE0);
	m_fbos[0].BindTex(0);
	glActiveTexture(GL_TEXTURE1);
	m_fbos[2].BindTex(0);
	glActiveTexture(GL_TEXTURE2);
	m_fbos[0].BindTex(1);
	m_compose.use();
	m_compose.setUniformVar1f("ratio", ratio);
	m_compose.setUniformVar1f("focalPoint", focalPoint);
	m_compose.setUniformVar1f("focalArea", focalArea);
	m_compose.setUniformVar1f("near", m_transforms->near());
	m_compose.setUniformVar1f("far", m_transforms->far());
	glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, wndSize[0], wndSize[1]);
	// geomQuad still bound here
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, (char *)NULL);
	m_geomQuad.release();

	glEnable(GL_DEPTH_TEST);

}

void Graphene::Impl::modifier(Keys::Modifier mod, bool down) {
	switch (mod) {
		case Keys::SHIFT: m_eventHandler->modifier()->shift() = down; break;
		case Keys::CTRL : m_eventHandler->modifier()->ctrl()  = down; break;
		case Keys::ALT  : m_eventHandler->modifier()->alt()   = down; break;
		default:          m_eventHandler->modifier()->altgr() = down;
	}
}

void Graphene::Impl::setCameraControl(std::string control) {
	m_camera->setControl(m_camControls[control]);

}

void Graphene::Impl::setOrtho(bool ortho) {
	m_camera->setOrtho(ortho);
}

	/*
void Graphene::Impl::updateState() {
	// update viewport and matrices
	int w,h;
	std::tie(w,h) = m_backend->getViewportSize();
	m_transforms->viewport   = glm::ivec4(0, 0, w, h);
	m_transforms->modelview  = m_camera->getModelViewMatrix();
	m_transforms->normal     = glm::inverseTranspose(glm::mat3(m_transforms.modelviewMat));
	m_transforms->projection = glm::mat4(1.f);//TODO: fill
}
	*/

#ifdef ENABLE_SCREENCAST
void Graphene::Impl::startScreencast(fs::path outputFile) {
	std::cout << "Recording to: " + outputFile.string() + "..." << std::endl;
	//updateState();
	//TODO: m_backend->setResizable(false);
	
	m_duration = std::chrono::duration<long int>::zero();
	m_lastUpdate = std::chrono::system_clock::now();
	m_status = m_backend->getStatus();

	m_scInfo.recWidth = m_transforms->viewport()[2];
	m_scInfo.recHeight = m_transforms->viewport()[3];
	m_scInfo.recWidth -= m_scInfo.recWidth % 16;
	m_scInfo.recHeight -= m_scInfo.recHeight % 16;
	m_scInfo.recording = true;
	m_scInfo.paused = false;
	m_scInfo.thread = std::shared_ptr<std::thread>(new std::thread(std::bind(&Graphene::Impl::encodeScreencast, this, outputFile)));
}

void Graphene::Impl::pauseScreencast() {
	m_scInfo.paused = true;
	m_status->set("Recording...  Paused.");
}

void Graphene::Impl::resumeScreencast() {
	m_scInfo.paused = false;
	m_lastUpdate = std::chrono::system_clock::now();
}

void Graphene::Impl::stopScreencast() {
	m_scInfo.recording = false;
	m_scInfo.thread->join();
	m_scInfo.thread.reset();
	m_duration = std::chrono::duration<long int>::zero();
	//TODO: m_backend->setResizable(true);
	m_status->set("Recording...  Stopped.");
}

void Graphene::Impl::encodeScreencast(fs::path outputFile) {
	std::string cmd = "x264 --fps "+lexical_cast<std::string>(m_fps)+" -o \""+outputFile.string()+"\" --input-res "+lexical_cast<std::string>(m_scInfo.recWidth)+"x"+lexical_cast<std::string>(m_scInfo.recHeight)+" --input-csp rgb -";
	std::cout << cmd << "\n";
	FILE* proc = popen(cmd.c_str(), "w");
	std::chrono::milliseconds dura(10);
	while (m_scInfo.recording || m_scInfo.queue.size()) {
		if (m_scInfo.paused || !m_scInfo.queue.size()) {
			std::this_thread::sleep_for(dura);
			continue;
		}
		auto now = std::chrono::system_clock::now();
		m_duration += now - m_lastUpdate;
		m_lastUpdate = now;
		printDuration();
		m_scInfo.queueMutex.lock();
		unsigned char* px = m_scInfo.queue.front();
		m_scInfo.queue.pop();
		m_scInfo.queueMutex.unlock();
		unsigned char* pxFlip = new unsigned char[m_scInfo.recWidth*m_scInfo.recHeight*3];
		for (int i=0; i<m_scInfo.recHeight; ++i) {
			unsigned char* oS = &(px[i*m_scInfo.recWidth*3]);
			unsigned char* oT = &(pxFlip[(m_scInfo.recHeight-i-1)*m_scInfo.recWidth*3]);
			memcpy(oT, oS, m_scInfo.recWidth*3*sizeof(unsigned char));
		}
		fwrite((void*)pxFlip, sizeof(unsigned char), m_scInfo.recWidth*m_scInfo.recHeight*3, proc);
		delete [] px;
		delete [] pxFlip;
	}
	pclose(proc);
}

void Graphene::Impl::printDuration() {
	long int h = std::chrono::duration_cast<std::chrono::hours>(m_duration).count();
	long int m = std::chrono::duration_cast<std::chrono::minutes>(m_duration).count();
	long int s = std::chrono::duration_cast<std::chrono::seconds>(m_duration).count();
	long int ms = std::chrono::duration_cast<std::chrono::milliseconds>(m_duration).count();
	std::stringstream str;
	str.width(2);
	str.fill('0');
	str << h << ":";
	str.width(2);
	str << m << ":";
	str.width(2);
	str << s << ":";
	str.width(2);
	str << (ms / 100);
	m_status->set("Recording...  "+str.str());
}
#endif // ENABLE_SCREENCAST

} // FW
