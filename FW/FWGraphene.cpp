/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */

#include <include/config.h>

#include "FWGraphene.h"

#include <iostream>
#include <map>
#include <tuple>
#include <queue>
#include <thread>
#include <mutex>
#include <chrono>

#include <FW/View/ViewCamera.h>
#include <FW/View/ViewOrbitCameraControl.h>
#include <FW/View/ViewFlyCameraControl.h>
#include <FW/View/ViewTransforms.h>
using namespace FW::View;

#include <FW/FWVisualizer.h>


#include <Library/Buffer/Texture.h>
#include <Library/Buffer/HdrFile.h>
#include <Library/Buffer/FBO.h>
#include <Library/Shader/ShaderProgram.h>
#include <Library/Buffer/Geometry.h>

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

struct EnvMap {
	Buffer::HdrFile diffuse;
	Buffer::HdrFile specular;
};

struct Graphene::Impl {
		Impl(GUI::Backend::Ptr backend, FW::Events::EventHandler::Ptr eventHandler, bool singleMode, bool noEffects, std::string hdrPath);
		virtual ~Impl();

		void initTransforms();
		void initEffects();
		void updateEffects(int width, int height);

		int run(int fps);
		void exit();

		void         addFactory(std::string name, Factory::Ptr factory);
		Factory::Ptr getFactory(std::string name);
		bool         hasFactory(std::string name);
		void         addVisualizer(std::string factoryName, std::string visName);
		bool         hasVisualizer(std::string visName);
		void         removeVisualizer(std::string visName);

		void         render();
		void         renderTonemap();
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

		void loadHDRMaps(std::string hdrPath);


		GUI::Backend::Ptr                         m_backend;
		FW::Events::EventHandler::Ptr             m_eventHandler;

		bool                                      m_singleMode;
		bool                                      m_noEffects;
		bool                                      m_hdr;

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

		std::vector<Buffer::FBO>                  m_fbos;
		Shader::ShaderProgram                     m_gaussH;
		Shader::ShaderProgram                     m_gaussV;
		Shader::ShaderProgram                     m_compose;
		Shader::ShaderProgram                     m_tonemap;
		Buffer::Geometry                          m_geomQuad;

		std::map<std::string, EnvMap>             m_envMaps;
		std::map<std::string, EnvTex>             m_envTextures;
		std::string                               m_crtMap;
};


/// GRAPHENE ///


Graphene::Graphene(GUI::Backend::Ptr backend, FW::Events::EventHandler::Ptr eventHandler, bool singleMode, bool noEffects, std::string hdrPath) {
	m_impl = std::shared_ptr<Impl>(new Impl(backend, eventHandler, singleMode, noEffects, hdrPath));
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


Graphene::Impl::Impl(GUI::Backend::Ptr backend, FW::Events::EventHandler::Ptr eventHandler, bool singleMode, bool noEffects, std::string hdrPath) : m_backend(backend), m_eventHandler(eventHandler), m_singleMode(singleMode), m_noEffects(noEffects), m_hdr(false) {
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

	if (hdrPath != "") loadHDRMaps(hdrPath);
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

	auto groupHDR = main->add<Section>("HDR Rendering", "groupHDR");
	auto exposure = groupHDR->add<Range>("Exposure", "exposure");
	exposure->setDigits(2);
	exposure->setMin(0.0);
	exposure->setMax(2.0);
	exposure->setValue(0.6);
	exposure->disable();
	if (m_envMaps.size()) {
		auto choiceHDR = groupHDR->add<Choice>("Environment Map", "envMap");
		for (const auto& m : m_envMaps) {
			choiceHDR->add(m.first, m.first);
		}
		m_crtMap = choiceHDR->value();
		choiceHDR->setCallback([&] (const std::string& m) { m_crtMap = m; });
	}
}

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
	m_gaussH.addShaders(std::string(GLSL_PREFIX)+"effectsQuad.vert", std::string(GLSL_PREFIX)+"effectsGaussH.frag");
	m_gaussV.addShaders(std::string(GLSL_PREFIX)+"effectsQuad.vert", std::string(GLSL_PREFIX)+"effectsGaussV.frag");
	m_compose.addShaders(std::string(GLSL_PREFIX)+"effectsQuad.vert", std::string(GLSL_PREFIX)+"effectsCompose.frag");
	m_tonemap.addShaders(std::string(GLSL_PREFIX)+"effectsQuad.vert", std::string(GLSL_PREFIX)+"effectsTonemap.frag");
	m_gaussH.link();
	m_gaussV.link();
	m_compose.link();
	m_tonemap.link();
	m_compose.use();
	m_compose.setUniformVar1i("Tex0", 0);
	m_compose.setUniformVar1i("Tex1", 1);
	m_compose.setUniformVar1i("Tex2", 2);
	m_tonemap.use();
	m_tonemap.setUniformVar1i("Tex0", 0);
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
	m_fbos[0].setSize(width, height);
	//m_fbos[0].AttachRender(GL_DEPTH_COMPONENT24);
	m_fbos[0].attachTexture(GL_RGBA32F_ARB);
	m_fbos[0].attachTexture(GL_DEPTH_COMPONENT24);
	for (unsigned int i=1; i<3; ++i) {
		m_fbos[i].setSize(int(width/2), int(height/2));
		m_fbos[i].attachTexture(GL_RGBA32F_ARB, GL_LINEAR);
		// clamping to preserve post-pro from blur leaking
		m_fbos[i].bindTex();
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	}
	Buffer::FBO::unbind();
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
	if (!m_noEffects && !m_fbos.size()) initEffects();
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
	VisualizerHandle::Ptr fwHandle(new VisualizerHandle(visName, transforms, m_eventHandler, m_camera->getPickRay(), &m_envTextures, m_envTextures.size() ? &m_crtMap : nullptr));
	auto guiHandle = m_backend->addVisualizer(visName);
	if (!guiHandle) return;
	vis->setHandles(fwHandle, guiHandle);
	vis->setProgressBarPool(m_backend->getProgressBarPool());
	vis->init();
	m_visualizer[visName] = vis;
	if (vis->isHDR()) {
		if (!m_hdr) {
			auto main = m_backend->getMainSettings();
			main->get<Range>({"groupHDR", "exposure"})->enable();
		}
		m_hdr = true;
	}
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
	if (!m_hdr) return;
	m_hdr = false;
	for (const auto& vis : m_visualizer) {
		if (vis.second->isHDR()) {
			m_hdr = true;
			break;
		}
	}
	if (!m_hdr) {
		auto main = m_backend->getMainSettings();
		main->get<Range>({"groupHDR", "exposure"})->disable();
	}
}

void Graphene::Impl::render() {
	float blur = 0.f;
	if (!m_noEffects && m_visualizer.size()) {
		blur = m_backend->getMainSettings()->get<Range>({"groupEffects", "groupFOD", "blur"})->value();
	}

	if (m_hdr || blur > 0.f) {
		m_fbos[0].bindOutput();
	}

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

	if (m_hdr && blur == 0.f) {
		renderTonemap();
	}

	if (blur > 0.f) {
		renderBlur(blur);
	}

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

void Graphene::Impl::renderTonemap() {
	auto wndSize = m_transforms->viewport().tail(2);
	auto main = m_backend->getMainSettings();
	float exposure = main->get<Range>({"groupHDR", "exposure"})->value();
	glDisable(GL_DEPTH_TEST);

	// compose
	Buffer::FBO::unbind();
	glActiveTexture(GL_TEXTURE0);
	m_fbos[0].bindTex(0);
	m_tonemap.use();
	m_tonemap.setUniformVar1f("exposure", exposure);
	glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, wndSize[0], wndSize[1]);
	m_geomQuad.bind();
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, (char *)NULL);
	m_geomQuad.release();

	glEnable(GL_DEPTH_TEST);
}

void Graphene::Impl::renderBlur(float ratio) {
	auto wndSize = m_transforms->viewport().tail(2);
	auto main = m_backend->getMainSettings();
	float focalPoint = main->get<Range>({"groupEffects", "groupFOD", "focalPoint"})->value();
	float focalArea  = main->get<Range>({"groupEffects", "groupFOD", "focalArea"})->value();
	float exposure   = main->get<Range>({"groupHDR", "exposure"})->value();
	int ortho = static_cast<int>(m_camera->getOrtho());
	glDisable(GL_DEPTH_TEST);

	// horizontal gauss
	m_fbos[1].bindOutput();
	m_fbos[0].bindTex();
	m_gaussH.use();
	glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0,0, wndSize[0]/2, wndSize[1]/2); // downsampling
	m_geomQuad.bind();
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, (char *)NULL);

	// vertical gauss
	m_fbos[2].bindOutput();
	m_fbos[1].bindTex();
	m_gaussV.use();
	glClear(GL_COLOR_BUFFER_BIT);
	// geomQuad still bound here
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, (char *)NULL);

	// compose
	Buffer::FBO::unbind();
	glActiveTexture(GL_TEXTURE0);
	m_fbos[0].bindTex(0);
	glActiveTexture(GL_TEXTURE1);
	m_fbos[2].bindTex(0);
	glActiveTexture(GL_TEXTURE2);
	m_fbos[0].bindTex(1);
	m_compose.use();
	m_compose.setUniformVar1i("ortho", ortho);
	m_compose.setUniformVar1f("ratio", ratio);
	m_compose.setUniformVar1f("focalPoint", focalPoint);
	m_compose.setUniformVar1f("focalArea", focalArea);
	m_compose.setUniformVar1f("near", m_transforms->near());
	m_compose.setUniformVar1f("far", m_transforms->far());
	m_compose.setUniformVar1f("exposure", exposure);
	m_compose.setUniformVar1i("tonemap", static_cast<int>(m_hdr));
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

void Graphene::Impl::loadHDRMaps(std::string hdrPath) {
	fs::path path(hdrPath);
	if (!fs::exists(path)) return;

	std::vector<std::string> spec, diff;
	fs::directory_iterator dirIt(path), endIt;
	for (; dirIt != endIt; ++dirIt) {
		fs::path p = dirIt->path();
		if (fs::is_directory(p)) continue;
		std::string filename = p.filename().string();

		boost::smatch what;
		boost::regex pattern("(\\w+)_specular.hdr");
		if (boost::regex_match(filename, what, pattern)) {
			spec.push_back(what[1]);
			continue;
		}
		pattern = boost::regex("(\\w+)_diffuse.hdr");
		if (boost::regex_match(filename, what, pattern)) {
			diff.push_back(what[1]);
		}
	}
	std::sort(spec.begin(), spec.end());
	std::sort(diff.begin(), diff.end());
	std::vector<std::string> maps = Algorithm::setUnion(spec, diff);

	for (const auto& m : maps) {
		fs::path pDiff = path / (m + "_diffuse.hdr");
		fs::path pSpec = path / (m + "_specular.hdr");
		EnvMap envMap;
		envMap.diffuse.load(pDiff.string());
		envMap.specular.load(pSpec.string());
		EnvTex envTex;
		envTex.diffuse  = Buffer::Texture::Ptr(new Buffer::Texture(GL_RGB16F_ARB, envMap.diffuse.width(), envMap.diffuse.height(), envMap.diffuse.data()));
		envTex.specular = Buffer::Texture::Ptr(new Buffer::Texture(GL_RGB16F_ARB, envMap.specular.width(), envMap.specular.height(), envMap.specular.data()));
		m_envMaps[m] = envMap;
		m_envTextures[m] = envTex;
	}
}

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
