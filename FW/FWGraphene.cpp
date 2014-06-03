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

#include "FWGBuffer.h"

#include <FW/FWVisualizer.h>


#include <Library/Buffer/Texture.h>
#include <Library/Buffer/HdrFile.h>
#include <Library/Shader/ShaderProgram.h>
#include <Library/Buffer/Geometry.h>
#include <Library/Random/RNG.h>
using Random::RNG;

#define DEBUG_SHADER

namespace FW {

#ifdef ENABLE_SCREENCAST
struct ScreencastInfo {
	bool recording;
	bool paused;
	std::shared_ptr<std::thread> thread;
	std::queue<unsigned char*> queue;
	std::mutex queueMutex;
	int recWidth;
	int recHeight;
};
#endif // ENABLE_SCREENCAST


struct  Graphene::Impl {
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
	void         renderGeometryPass();
	void         renderLightPass();
	void         renderBlurPass();
	void         renderSSAOPass();
	void         renderFullQuad(int width, int height);

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


	GUI::Backend::Ptr m_backend;
	FW::Events::EventHandler::Ptr m_eventHandler;

	bool m_singleMode;
	bool m_noEffects;

	std::map<std::string, Factory::Ptr> m_factories;
	std::map<std::string, Visualizer::Ptr> m_visualizer;

	Transforms::Ptr m_transforms;
	std::map<std::string, CameraControl::Ptr> m_camControls;
	Camera::Ptr m_camera;
	int m_fps;
#ifdef ENABLE_SCREENCAST
	ScreencastInfo m_scInfo;
	std::chrono::system_clock::time_point m_lastUpdate;
	std::chrono::system_clock::duration m_duration;
	GUI::Status::Ptr m_status;
#endif // ENABLE_SCREENCAST

	GBuffer m_gbuffer;
	Shader::ShaderProgram m_geomPass;
	Shader::ShaderProgram m_lightPass;
	Shader::ShaderProgram m_blurPass;
	Shader::ShaderProgram m_ssaoPass;
	Buffer::Geometry m_geomQuad;

	std::map<std::string, EnvTex> m_envTextures;
	std::string m_crtMap;

	// fps computation
	bool m_showFPS;
	std::chrono::system_clock::time_point m_lastFPSComp;
	unsigned int m_frameCount;
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


Graphene::Impl::Impl(GUI::Backend::Ptr backend, FW::Events::EventHandler::Ptr eventHandler, bool singleMode, bool noEffects, std::string hdrPath) : m_backend(backend), m_eventHandler(eventHandler), m_singleMode(singleMode), m_noEffects(noEffects), m_showFPS(false), m_frameCount(0) {
	m_lastFPSComp = std::chrono::system_clock::now();
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
	m_scInfo.paused    = false;
	m_scInfo.thread    = std::shared_ptr<std::thread>();
#endif // ENABLE_SCREENCAST

	if (hdrPath != "") loadHDRMaps(hdrPath);
	initTransforms();
}

Graphene::Impl::~Impl() {
}

void Graphene::Impl::initTransforms() {
	m_transforms = View::Transforms::Ptr(new View::Transforms());
	auto  glSize = m_backend->getGLSize();
	m_transforms->viewport() = Eigen::Vector4i(0, 0, glSize[0], glSize[1]);
	m_eventHandler->registerReceiver<void (int, int)>("WINDOW_RESIZE", "mainapp", std::function<void (int, int)>([&] (int w, int h) {
	                                                                                                                m_transforms->viewport()[2] = w;
	                                                                                                                m_transforms->viewport()[3] = h;
	                                                                                                                m_camera->updateTransforms();
																																					 }));
	m_camControls["orbit"] = OrbitCameraControl::Ptr(new OrbitCameraControl());
	m_camControls["fly"]   = FlyCameraControl::Ptr(new FlyCameraControl());
	m_camera = Camera::Ptr(new Camera(m_camControls["orbit"], m_eventHandler, Transforms::WPtr(m_transforms)));
	// init settings
	auto  main     = m_backend->getMainSettings();
	auto  groupNav = main->add<Section>("Navigation", "groupNavigation");
	auto  ccChoice = groupNav->add<Choice>("Camera Control:", "camControl");
	ccChoice->add("orbit", "Orbit Control");
	ccChoice->add("fly", "Fly Control");
	ccChoice->setCallback(std::bind(&Graphene::Impl::setCameraControl, this, std::placeholders::_1));
	auto  groupRender = main->add<Section>("Rendering", "groupRendering");
	groupRender->add<Color>("Background: ", "background")->setValue(Eigen::Vector4f(1.f, 1.f, 1.f, 1.f));
	auto  projection  = groupRender->add<Choice>("Projection:");
	projection->add("perspective", "Perspective");
	projection->add("ortho", "Orthographic");
	projection->setCallback([&] (std::string mode) {
	                           setOrtho(mode == "ortho");
									});
	if (m_singleMode) {
		groupNav->collapse();
		groupRender->collapse();
	}

	auto  groupHDR = groupRender->add<Section>("Image Based Lighting", "groupHDR");
	groupHDR->add<Range>("Exposure", "exposure")->setDigits(2).setMin(0.0).setMax(2.0).setValue(0.6);
	if (m_envTextures.size()) {
		auto  choiceHDR = groupHDR->add<Choice>("Environment Map", "envMap");
		for (const auto& m : m_envTextures) {
			choiceHDR->add(m.first, m.first);
		}
		m_crtMap = choiceHDR->value();
		choiceHDR->setCallback([&] (const std::string& m) {
		                          m_crtMap = m;
									  });
	}

	auto  fod = groupRender->add<Group>("Field Of Depth", "groupFOD");
	fod->add<Bool>("Blur Enabled", "blurEnabled")->setValue(true);
	fod->add<Bool>("Blooming Enabled", "bloomEnabled")->setValue(false);
	fod->add<Bool>("Bloom Only Blurred", "fodBloom")->setValue(true);
	fod->add<Range>("Blur Ratio", "blur")->setDigits(2).setMin(0.f).setMax(1.f).setValue(0.f);
	fod->add<Range>("Blooming", "bloom")->setDigits(2).setMin(0.f).setMax(5.f).setValue(0.f);
	fod->add<Range>("Bloom Threshold", "bloomCut")->setDigits(2).setMin(0.f).setMax(2.f).setValue(1.f);
	fod->add<Range>("Focal Point", "focalPoint")->setDigits(2).setMin(0.f).setMax(1.f).setValue(0.0f);
	fod->add<Range>("Focal Area", "focalArea")->setDigits(2).setMin(0.f).setMax(1.f).setValue(0.5f);

	auto  ssao = groupRender->add<Group>("Screen-Space Ambient Occlusion", "groupSSAO");
	ssao->add<Bool>("Enabled", "ssaoActive")->setValue(false);
	ssao->add<Range>("Factor", "ssaoFactor")->setDigits(2).setMin(0.01f).setMax(1.f).setValue(1.f);
	ssao->add<Range>("Radius", "radius")->setDigits(2).setMin(0.01f).setMax(5.f).setValue(0.5f);
	ssao->add<Range>("Exponent", "exponent")->setDigits(2).setMin(1).setMax(5).setValue(2);
	auto  ssaoSamples = ssao->add<Range>("Samples", "samples");
	ssaoSamples->setDigits(0).setMin(1).setMax(128);
	ssaoSamples->setValue(50);
	ssaoSamples->setCallback([&] (float) {
	                            auto wndSize = m_transforms->viewport().tail(2);
	                            updateEffects(wndSize[0], wndSize[1]);
									 });

#ifdef DEBUG_SHADER
	auto  debug = groupRender->add<Group>("Debug", "debug");
	debug->add<Bool>("Normals", "debugNormals")->setValue(false);;
	debug->add<Bool>("SSAO", "debugSSAO")->setValue(false);
	auto fps = debug->add<Bool>("FPS", "fps");
	fps->setValue(false);
	fps->setCallback([&] (bool state) { m_showFPS = state; });
#endif

	if (m_singleMode) groupRender->collapse();
}

void Graphene::Impl::initEffects() {
	// events
	m_eventHandler->registerReceiver<void (int, int, int, int)>("LEFT_DRAG", "mainapp", [&] (int dx, int dy, int x, int y) {
	                                                               if (!(m_eventHandler->modifier()->ctrl() && m_eventHandler->modifier()->shift()) ) return;
	                                                               float newVal = m_backend->getMainSettings()->get<Range>({"groupRendering", "groupFOD", "focalArea"})->value() + (-0.01f * dy);
	                                                               if (newVal > 1.f) newVal = 1.f;
	                                                               if (newVal < 0.f) newVal = 0.f;
	                                                               m_backend->getMainSettings()->get<Range>({"groupRendering", "groupFOD", "focalArea"})->setValue(newVal);
																					});
	m_eventHandler->registerReceiver<void (int, int, int, int)>("RIGHT_DRAG", "mainapp", [&] (int dx, int dy, int x, int y) {
	                                                               if (!(m_eventHandler->modifier()->ctrl() && m_eventHandler->modifier()->shift()) ) return;
	                                                               float newVal = m_backend->getMainSettings()->get<Range>({"groupRendering", "groupFOD", "focalPoint"})->value() + (-0.01f * dy);
	                                                               if (newVal > 1.f) newVal = 1.f;
	                                                               if (newVal < 0.f) newVal = 0.f;
	                                                               m_backend->getMainSettings()->get<Range>({"groupRendering", "groupFOD", "focalPoint"})->setValue(newVal);
																					});

	std::vector<Vector3f>  quadVertices;
	quadVertices.push_back(Vector3f(-1.f, -1.f, 0.f));
	quadVertices.push_back(Vector3f( 1.f, -1.f, 0.f));
	quadVertices.push_back(Vector3f( 1.f,  1.f, 0.f));
	quadVertices.push_back(Vector3f(-1.f,  1.f, 0.f));
	std::vector<GLuint>  quadIndices(6);
	quadIndices[0] = 0; quadIndices[1] = 1; quadIndices[2] = 2;
	quadIndices[3] = 0; quadIndices[4] = 2; quadIndices[5] = 3;
	m_geomQuad.init();
	m_geomQuad.setVertices(quadVertices);
	m_geomQuad.setIndices(quadIndices);
	m_geomQuad.upload();
	m_geomQuad.enableVertices();
	m_geomQuad.enableIndices();

// shaders and geometries
	m_geomPass.addShaders(std::string(GLSL_PREFIX) + "geomPass.vert", std::string(GLSL_PREFIX) + "geomPass.frag");
	m_lightPass.addShaders(std::string(GLSL_PREFIX) + "fullQuad.vert", std::string(GLSL_PREFIX) + "lightPass.frag");
	m_blurPass.addShaders(std::string(GLSL_PREFIX) + "fullQuad.vert", std::string(GLSL_PREFIX) + "blurPass.frag");
	m_ssaoPass.addShaders(std::string(GLSL_PREFIX) + "fullQuad.vert", std::string(GLSL_PREFIX) + "ssaoPass.frag");
	std::map<int, std::string>  outputMap;
	outputMap[0] = "outPos";
	outputMap[1] = "outCol";
	outputMap[2] = "outNrm";
	m_geomPass.link(outputMap);
	m_lightPass.link();
	outputMap.clear();
	outputMap[0] = "blur";
	outputMap[1] = "bloom";
	outputMap[2] = "ssao";
	m_blurPass.link(outputMap);
	m_ssaoPass.link();
	auto  wndSize = m_transforms->viewport().tail(2);
	updateEffects(wndSize[0], wndSize[1]);
	m_eventHandler->registerReceiver<void (int width, int height)>("WINDOW_RESIZE", "mainapp", std::bind(&Graphene::Impl::updateEffects, this, std::placeholders::_1, std::placeholders::_2));

	m_geomQuad.bindVertices(m_lightPass, "position");
	m_geomQuad.bindVertices(m_blurPass, "position");
	m_geomQuad.bindVertices(m_ssaoPass, "position");
}

void Graphene::Impl::updateEffects(int width, int height) {
	m_gbuffer.init(width, height);

	// update view ray
	float  aspect     = m_camera->getAspectRatio(width, height);
	float  tanHalfFov = std::tan(m_camera->getFieldOfView() / 2.f);

	// generate gaussian kernel
	int    kernelWidth = 9;
	float  sigma2      = 4.0f * 4.0f * 2.f;
	float  factor      = 1.f / (M_PI * sigma2);
	float  sum         = 0.f;
	float* weights     = new float[kernelWidth * kernelWidth];
	float* offsetsH    = new float[kernelWidth];
	float* offsetsV    = new float[kernelWidth];
	int    center      = kernelWidth / 2;
	for (int i = 0; i < kernelWidth; i++) {
		int  i2 = (i - center) * (i - center);
		for (int j = 0; j < kernelWidth; j++) {
			int    j2 = (j - center) * (j - center);
			float  w  = factor * exp(-(i2 + j2) / sigma2);
			weights[i * kernelWidth + j] = w;
			sum += w;
		}
		offsetsH[i] = (i - 4.0f) / float(width / 2.0f);
		offsetsV[i] = (i - 4.0f) / float(height / 2.0f);
	}
	for (int i = 0; i < kernelWidth; i++) {
		for (int j = 0; j < kernelWidth; j++) {
			weights[i * kernelWidth + j] /= sum;
		}
	}
	m_blurPass.use();
	m_blurPass.setUniformVar1f("weights", kernelWidth * kernelWidth, weights);
	m_blurPass.setUniformVar1f("offsetsH", kernelWidth, offsetsH);
	m_blurPass.setUniformVar1f("offsetsV", kernelWidth, offsetsV);
	// m_blurPass.setUniformVar1f("aspectRatio", aspect);
	// m_blurPass.setUniformVar1f("tanHalfFov", tanHalfFov);

	delete [] weights;
	delete [] offsetsH;
	delete [] offsetsV;

	// generate noise and random kernels for SSAO
	auto             main       = m_backend->getMainSettings();
	int              numSamples =  main->get<Range>({"groupRendering", "groupSSAO", "samples"})->value();
	auto             gen        = RNG::uniform01Gen<float>();
	Eigen::MatrixXf  samples(3, numSamples);
	for (int i = 0; i < numSamples; ++i) {
		Vector3f  pos(2.f * gen() - 1.f, 2.f * gen() - 1.f, gen());
		pos.normalize();
		pos *= gen();
		samples.col(i) = pos;
	}
	int    noiseSize = 4;
	float* noise     = new float[noiseSize * noiseSize * 2];
	for (int i = 0; i < noiseSize * noiseSize; ++i) {
		Vector2f  n(2.f * gen() - 1.f, 2.f * gen() - 1.f);
		n.normalize();
		noise[i * 2 + 0] = n[0];
		noise[i * 2 + 1] = n[1];
	}

	m_ssaoPass.use();
	m_ssaoPass.setUniformVec3("samples", samples.data(), numSamples);
	m_ssaoPass.setUniformVar1f("noise", noiseSize * noiseSize * 2, noise);
	m_ssaoPass.setUniformVar1i("numSamples", numSamples);
	// m_ssaoPass.setUniformVar1f("aspectRatio", aspect);
	// m_ssaoPass.setUniformVar1f("tanHalfFov", tanHalfFov);

	delete [] noise;

	// m_lightPass.use();
	// m_lightPass.setUniformVar1f("aspectRatio", aspect);
	// m_lightPass.setUniformVar1f("tanHalfFov", tanHalfFov);
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
	auto  handle = m_backend->addFactory(name);
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
	if (!m_noEffects && !m_gbuffer.initialized()) initEffects();
	if (hasVisualizer(visName)) {
		m_backend->getLog()->error("Visualizer already exists");
		return;
	}
	if (!hasFactory(factoryName)) {
		m_backend->getLog()->error("Factory does not exist");
		return;
	}
	auto  vis = getFactory(factoryName)->addVisualizer();
	if (!vis) return;
	View::Transforms::WPtr  transforms(m_transforms);
	VisualizerHandle::Ptr   fwHandle(new VisualizerHandle(visName, transforms, m_eventHandler, m_camera->getPickRay()));
	auto  guiHandle = m_backend->addVisualizer(visName);
	if (!guiHandle) return;
	vis->setHandles(fwHandle, guiHandle);
	vis->setProgressBarPool(m_backend->getProgressBarPool());
	vis->init();
	m_visualizer[visName] = vis;
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
	if (m_showFPS) {
		++m_frameCount;
		auto now = std::chrono::system_clock::now();
		auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastFPSComp).count();
		if (dur > 500) {
			float fps = m_frameCount * 1000.f / dur;
			m_lastFPSComp = now;
			m_frameCount = 0;
			auto status = m_backend->getStatus();
			status->set(lexical_cast<std::string>(fps));
		}
	}
	if (!m_gbuffer.initialized()) {
		auto      main    = m_backend->getMainSettings();
		Vector4f  bg      = m_backend->getBackgroundColor();
		glClearColor(bg[0], bg[1], bg[2], 1.f);
		glClear(GL_COLOR_BUFFER_BIT);
		return;
	}

	// determine bounding box
	BoundingBox  bbox;
	if (m_singleMode) {
		for (const auto& vis : m_visualizer) {
			bbox.extend(vis.second->boundingBox());
		}
	} else {
		auto  names = m_backend->getActiveVisualizerNames();
		for (const auto& name : names) {
			bbox.extend(m_visualizer[name]->boundingBox());
		}
	}
	float     farDist = 0.f, nearDist = std::numeric_limits<float>::max(), dist;
	Vector3f  camPos  = m_camera->getPosition();
	Vector3f  camDir  = (m_camera->getLookAt() - camPos).normalized();
	dist     = (bbox.corner(BoundingBox::BottomLeftFloor) - camPos).dot(camDir);
	farDist  = std::max(farDist, dist); nearDist = std::min(nearDist, dist);
	dist     = (bbox.corner(BoundingBox::BottomRightFloor) - camPos).dot(camDir);
	farDist  = std::max(farDist, dist); nearDist = std::min(nearDist, dist);
	dist     = (bbox.corner(BoundingBox::TopLeftFloor) - camPos).dot(camDir);
	farDist  = std::max(farDist, dist); nearDist = std::min(nearDist, dist);
	dist     = (bbox.corner(BoundingBox::TopRightFloor) - camPos).dot(camDir);
	farDist  = std::max(farDist, dist); nearDist = std::min(nearDist, dist);
	dist     = (bbox.corner(BoundingBox::BottomLeftCeil) - camPos).dot(camDir);
	farDist  = std::max(farDist, dist); nearDist = std::min(nearDist, dist);
	dist     = (bbox.corner(BoundingBox::BottomRightCeil) - camPos).dot(camDir);
	farDist  = std::max(farDist, dist); nearDist = std::min(nearDist, dist);
	dist     = (bbox.corner(BoundingBox::TopLeftCeil) - camPos).dot(camDir);
	farDist  = std::max(farDist, dist); nearDist = std::min(nearDist, dist);
	dist     = (bbox.corner(BoundingBox::TopRightCeil) - camPos).dot(camDir);
	farDist  = std::max(farDist, dist); nearDist = std::min(nearDist, dist);
	nearDist = std::max(nearDist, 0.05f);
	//nearDist -= 0.01;
	farDist += 10.f;
	m_camera->setClipping(nearDist, farDist);
	m_lightPass.setUniformVar1f("near", m_transforms->near());
	m_lightPass.setUniformVar1f("far", m_transforms->far());

	auto  main        = m_backend->getMainSettings();
	bool  ssaoActive  =  main->get<Bool>({"groupRendering", "groupSSAO", "ssaoActive"})->value();
	bool  blurActive  = main->get<Bool>({"groupRendering", "groupFOD", "blurEnabled"})->value();
	bool  bloomActive = main->get<Bool>({"groupRendering", "groupFOD", "bloomEnabled"})->value();

	renderGeometryPass();
	if (ssaoActive) {
		renderSSAOPass();
	}
	if (ssaoActive || blurActive) renderBlurPass();
	renderLightPass();


#ifdef ENABLE_SCREENCAST
	if (m_scInfo.recording && !m_scInfo.paused) {
		unsigned char* px = new unsigned char[3 * m_scInfo.recWidth * m_scInfo.recHeight];
		glReadPixels(0, 0, m_scInfo.recWidth, m_scInfo.recHeight, GL_RGB, GL_UNSIGNED_BYTE, (void*)px);
		m_scInfo.queueMutex.lock();
		m_scInfo.queue.push(px);
		m_scInfo.queueMutex.unlock();
	}
#endif // ENABLE_SCREENCAST
}

void Graphene::Impl::renderGeometryPass() {
	auto      main    = m_backend->getMainSettings();
	Vector4f  bg      = m_backend->getBackgroundColor();

	auto  diff        = m_envTextures[m_crtMap].diffuse;
	auto  spec        = m_envTextures[m_crtMap].specular;
#ifdef DEBUG_SHADER
	int  debugNormals = main->get<Bool>({"groupRendering", "debug", "debugNormals"})->value() ? 1 : 0;
#endif

	//Vector3f  viewDir = (m_camera->getLookAt() - m_camera->getPosition()).normalized();
	Vector3f  camPos = m_camera->getPosition();
	m_gbuffer.bindGeomPass(m_geomPass, bg, diff, spec);
	m_geomPass.use();
	m_geomPass.setUniformMat4("mvM", m_transforms->modelview().data());
	m_geomPass.setUniformMat4("prM", m_transforms->projection().data());
	m_geomPass.setUniformVec3("camPos", camPos.data());
#ifdef DEBUG_SHADER
	m_geomPass.setUniformVar1i("debugNormals", debugNormals);
#endif


	glDepthMask(GL_TRUE);
	glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	if (m_singleMode) {
		for (const auto& vis : m_visualizer) {
			vis.second->waitForTasks();
			vis.second->render(m_geomPass);
		}
	} else {
		auto  names = m_backend->getActiveVisualizerNames();
		for (const auto& name : names) {
			m_visualizer[name]->waitForTasks();
			m_visualizer[name]->render(m_geomPass);
		}
	}
	glDisable(GL_DEPTH_TEST);
}

void Graphene::Impl::renderLightPass() {
	auto  main = m_backend->getMainSettings();

	glDepthMask(GL_FALSE);

	float  exposure    = main->get<Range>({"groupRendering", "groupHDR", "exposure"})->value();
	auto   wndSize     = m_transforms->viewport().tail(2);
	int    ortho       = static_cast<int>(m_camera->getOrtho());

	bool   blurActive  = main->get<Bool>({"groupRendering", "groupFOD", "blurEnabled"})->value();
	bool   bloomActive = main->get<Bool>({"groupRendering", "groupFOD", "bloomEnabled"})->value();
	bool   fodBloom    = main->get<Bool>({"groupRendering", "groupFOD", "fodBloom"})->value();
	bool   ssaoActive  =  main->get<Bool>({"groupRendering", "groupSSAO", "ssaoActive"})->value();
	float  ssaoFactor  =  main->get<Range>({"groupRendering", "groupSSAO", "ssaoFactor"})->value();

	float  ratio       = main->get<Range>({"groupRendering", "groupFOD", "blur"})->value();
	float  bloom       = main->get<Range>({"groupRendering", "groupFOD", "bloom"})->value();
	float  focalPoint  = main->get<Range>({"groupRendering", "groupFOD", "focalPoint"})->value();
	float  focalArea   = main->get<Range>({"groupRendering", "groupFOD", "focalArea"})->value();
#ifdef DEBUG_SHADER
	int  debugNormals  = main->get<Bool>({"groupRendering", "debug", "debugNormals"})->value() ? 1 : 0;
	int  debugSSAO     = main->get<Bool>({"groupRendering", "debug", "debugSSAO"})->value() ? 1 : 0;
#endif

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	m_gbuffer.bindLightPass(m_lightPass);
	glClear(GL_COLOR_BUFFER_BIT);

	m_lightPass.use();
	m_lightPass.setUniformVar1i("ortho", ortho);
	m_lightPass.setUniformVar1f("ratio", ratio);
	m_lightPass.setUniformVar1f("bloom", bloom);
	m_lightPass.setUniformVar1f("ssaoFactor", ssaoFactor);
	m_lightPass.setUniformVar1f("focalPoint", focalPoint);
	m_lightPass.setUniformVar1f("focalArea", focalArea);
	m_lightPass.setUniformVar1f("near", m_transforms->near());
	m_lightPass.setUniformVar1f("far", m_transforms->far());
	m_lightPass.setUniformVar1f("exposure", exposure);
	m_lightPass.setUniformVar1i("blurActive", blurActive ? 1 : 0);
	m_lightPass.setUniformVar1i("bloomActive", bloomActive ? 1 : 0);
	m_lightPass.setUniformVar1i("ssaoActive", ssaoActive ? 1 : 0);
	m_lightPass.setUniformVar1i("fodBloom", fodBloom ? 1 : 0);
#ifdef DEBUG_SHADER
	m_lightPass.setUniformVar1i("debugNormals", debugNormals);
	m_lightPass.setUniformVar1i("debugSSAO", debugSSAO);
#endif
	renderFullQuad(wndSize[0], wndSize[1]);
}

void Graphene::Impl::renderBlurPass() {
	auto  wndSize = m_transforms->viewport().tail(2);
	m_gbuffer.bindBlurPass(m_blurPass);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT);

	auto   main        = m_backend->getMainSettings();
	float  bloomCut    = main->get<Range>({"groupRendering", "groupFOD", "bloomCut"})->value();
	bool   ssaoActive  =  main->get<Bool>({"groupRendering", "groupSSAO", "ssaoActive"})->value();
	bool   blurActive  = main->get<Bool>({"groupRendering", "groupFOD", "blurEnabled"})->value();
	bool   bloomActive = main->get<Bool>({"groupRendering", "groupFOD", "bloomEnabled"})->value();
#ifdef DEBUG_SHADER
	int    debugSSAO   = main->get<Bool>({"groupRendering", "debug", "debugSSAO"})->value() ? 1 : 0;
#endif

	m_blurPass.use();
	m_blurPass.setUniformVar1i("ssaoActive", ssaoActive);
	m_blurPass.setUniformVar1i("blurActive", blurActive);
	m_blurPass.setUniformVar1i("bloomActive", bloomActive);
#ifdef DEBUG_SHADER
	m_blurPass.setUniformVar1i("debugSSAO", debugSSAO);
#endif
	m_blurPass.setUniformVar1f("bloomCut", bloomCut);
	renderFullQuad(wndSize[0], wndSize[1]);
}

void Graphene::Impl::renderSSAOPass() {
	auto  wndSize = m_transforms->viewport().tail(2);
	m_gbuffer.bindSSAOPass(m_ssaoPass);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT);

	auto   main   = m_backend->getMainSettings();
	float  radius = main->get<Range>({"groupRendering", "groupSSAO", "radius"})->value();
	float  power  = main->get<Range>({"groupRendering", "groupSSAO", "exponent"})->value();

	m_ssaoPass.use();
	m_ssaoPass.setUniformMat4("mvM", m_transforms->modelview().data());
	m_ssaoPass.setUniformMat4("prM", m_transforms->projection().data());
	// m_ssaoPass.setUniformMat3("nmM", m_transforms->normal().data());
	m_ssaoPass.setUniformVar1f("radius", radius);
	m_ssaoPass.setUniformVar1f("power", power);
	renderFullQuad(wndSize[0], wndSize[1]);
}

void Graphene::Impl::renderFullQuad(int width, int height) {
	glDisable(GL_DEPTH_TEST);

	// store blend mode and disable blending
	GLboolean blendEnabled;
	glGetBooleanv(GL_BLEND, &blendEnabled);
	glDisable(GL_BLEND);

	glViewport(0, 0, width, height);
	m_geomQuad.bind();
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, (char*)NULL);
	m_geomQuad.release();

	// restore blend mode
	if (blendEnabled) glEnable(GL_BLEND);

	glEnable(GL_DEPTH_TEST);
}

void Graphene::Impl::modifier(Keys::Modifier mod, bool down) {
	switch (mod) {
		case Keys::SHIFT: m_eventHandler->modifier()->shift() = down; break;
		case Keys::CTRL: m_eventHandler->modifier()->ctrl()   = down; break;
		case Keys::ALT: m_eventHandler->modifier()->alt()     = down; break;
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
	fs::path  path(hdrPath);
	if (!fs::exists(path)) return;

	std::vector<std::string>  spec, diff;
	fs::directory_iterator    dirIt(path), endIt;
	for (; dirIt != endIt; ++dirIt) {
		fs::path  p           = dirIt->path();
		if (fs::is_directory(p)) continue;
		std::string  filename = p.filename().string();

		boost::smatch  what;
		boost::regex   pattern("(\\w+)_specular.hdr");
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
	std::vector<std::string>  maps = Algorithm::setUnion(spec, diff);

	for (const auto& m : maps) {
		fs::path         pDiff = path / (m + "_diffuse.hdr");
		fs::path         pSpec = path / (m + "_specular.hdr");
		Buffer::HdrFile  diffuse;
		Buffer::HdrFile  specular;
		diffuse.load(pDiff.string());
		specular.load(pSpec.string());
		EnvTex  envTex;
		envTex.diffuse   = Buffer::Texture::Ptr(new Buffer::Texture(GL_RGB16F_ARB, diffuse.width(), diffuse.height(), diffuse.data()));
		envTex.specular  = Buffer::Texture::Ptr(new Buffer::Texture(GL_RGB16F_ARB, specular.width(), specular.height(), specular.data()));
		m_envTextures[m] = envTex;
	}
}

#ifdef ENABLE_SCREENCAST
void Graphene::Impl::startScreencast(fs::path outputFile) {
	std::cout << "Recording to: " + outputFile.string() + "..." << std::endl;
	// updateState();
	// TODO: m_backend->setResizable(false);

	m_duration          = std::chrono::duration<long int>::zero();
	m_lastUpdate        = std::chrono::system_clock::now();
	m_status            = m_backend->getStatus();

	m_scInfo.recWidth   = m_transforms->viewport()[2];
	m_scInfo.recHeight  = m_transforms->viewport()[3];
	m_scInfo.recWidth  -= m_scInfo.recWidth % 16;
	m_scInfo.recHeight -= m_scInfo.recHeight % 16;
	m_scInfo.recording  = true;
	m_scInfo.paused     = false;
	m_scInfo.thread     = std::shared_ptr<std::thread>(new std::thread(std::bind(&Graphene::Impl::encodeScreencast, this, outputFile)));
}

void Graphene::Impl::pauseScreencast() {
	m_scInfo.paused = true;
	m_status->set("Recording...  Paused.");
}

void Graphene::Impl::resumeScreencast() {
	m_scInfo.paused = false;
	m_lastUpdate    = std::chrono::system_clock::now();
}

void Graphene::Impl::stopScreencast() {
	m_scInfo.recording = false;
	m_scInfo.thread->join();
	m_scInfo.thread.reset();
	m_duration = std::chrono::duration<long int>::zero();
	// TODO: m_backend->setResizable(true);
	m_status->set("Recording...  Stopped.");
}

void Graphene::Impl::encodeScreencast(fs::path outputFile) {
	std::string  cmd = "x264 --fps " + lexical_cast<std::string>(m_fps) + " -o \"" + outputFile.string() + "\" --input-res " + lexical_cast<std::string>(m_scInfo.recWidth) + "x" + lexical_cast<std::string>(m_scInfo.recHeight) + " --input-csp rgb -";
	std::cout << cmd << "\n";
	FILE* proc       = popen(cmd.c_str(), "w");
	std::chrono::milliseconds  dura(10);
	while (m_scInfo.recording || m_scInfo.queue.size()) {
		if (m_scInfo.paused || !m_scInfo.queue.size()) {
			std::this_thread::sleep_for(dura);
			continue;
		}
		auto  now = std::chrono::system_clock::now();
		m_duration  += now - m_lastUpdate;
		m_lastUpdate = now;
		printDuration();
		m_scInfo.queueMutex.lock();
		unsigned char* px = m_scInfo.queue.front();
		m_scInfo.queue.pop();
		m_scInfo.queueMutex.unlock();
		unsigned char* pxFlip = new unsigned char[m_scInfo.recWidth * m_scInfo.recHeight * 3];
		for (int i = 0; i < m_scInfo.recHeight; ++i) {
			unsigned char* oS = &(px[i * m_scInfo.recWidth * 3]);
			unsigned char* oT = &(pxFlip[(m_scInfo.recHeight - i - 1) * m_scInfo.recWidth * 3]);
			memcpy(oT, oS, m_scInfo.recWidth * 3 * sizeof(unsigned char));
		}
		fwrite((void*)pxFlip, sizeof(unsigned char), m_scInfo.recWidth * m_scInfo.recHeight * 3, proc);
		delete [] px;
		delete [] pxFlip;
	}
	pclose(proc);
}

void Graphene::Impl::printDuration() {
	long int           h  = std::chrono::duration_cast<std::chrono::hours>(m_duration).count();
	long int           m  = std::chrono::duration_cast<std::chrono::minutes>(m_duration).count();
	long int           s  = std::chrono::duration_cast<std::chrono::seconds>(m_duration).count();
	long int           ms = std::chrono::duration_cast<std::chrono::milliseconds>(m_duration).count();
	std::stringstream  str;
	str.width(2);
	str.fill('0');
	str << h << ":";
	str.width(2);
	str << m << ":";
	str.width(2);
	str << s << ":";
	str.width(2);
	str << (ms / 100);
	m_status->set("Recording...  " + str.str());
}

#endif // ENABLE_SCREENCAST

} // FW
