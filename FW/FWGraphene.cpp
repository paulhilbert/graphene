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

#include <FW/FWVisualizer.h>

#ifdef USE_SPACENAV
#include <FW/Events/EventsSpaceNav.h>
#endif


namespace FW {


struct  Graphene::Impl {
    typedef harmont::camera::vec3_t vec3_t;

	Impl(GUI::Backend::Ptr backend, FW::Events::EventHandler::Ptr eventHandler, bool singleMode, const RenderParameters& renderParams, const ShadowParameters& shadowParams, std::string hdrPath);
	virtual ~Impl();

	//void initTransforms();
    void initRenderer();
    void initRenderProperties();

	int run(int fps);
	void exit();

	void         addFactory(std::string name, Factory::Ptr factory);
	Factory::Ptr getFactory(std::string name);
	bool         hasFactory(std::string name);
	void         addVisualizer(std::string factoryName, std::string visName);
	bool         hasVisualizer(std::string visName);
	void         removeVisualizer(std::string visName);

	void         render();

	void         modifier(Keys::Modifier mod, bool down);

	//void         setCameraControl(std::string control);
	//void         setOrtho(bool ortho);


    // framework
	GUI::Backend::Ptr m_backend;
	FW::Events::EventHandler::Ptr m_eventHandler;

	std::map<std::string, Factory::Ptr> m_factories;
	std::map<std::string, Visualizer::Ptr> m_visualizer;

    // rendering
    harmont::camera::ptr m_camera;
    harmont::deferred_renderer::ptr_t m_renderer;
    Eigen::Vector3f m_lightDir;
    RenderParameters m_rParams;
    ShadowParameters m_sParams;
    BoundingBox m_bbox;

    // special modii
	bool m_singleMode;

	// fps computation
	int m_fps;
	bool m_showFPS;
	std::chrono::system_clock::time_point m_lastFPSComp;
	unsigned int m_frameCount;

    // hdr map
    std::string m_hdrPath;

	//std::map<std::string, CameraControl::Ptr> m_camControls;

	//std::map<std::string, EnvTex> m_envTextures;
	//std::string m_crtMap;
	
#ifdef USE_SPACENAV
	FW::Events::SpaceNav::Ptr m_spaceNav;
#endif
};


/// GRAPHENE ///


Graphene::Graphene(GUI::Backend::Ptr backend, FW::Events::EventHandler::Ptr eventHandler, bool singleMode, const RenderParameters& renderParams, const ShadowParameters& shadowParams, std::string hdrPath) {
	m_impl = std::shared_ptr<Impl>(new Impl(backend, eventHandler, singleMode, renderParams, shadowParams, hdrPath));
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


Graphene::Impl::Impl(GUI::Backend::Ptr backend, FW::Events::EventHandler::Ptr eventHandler, bool singleMode, const RenderParameters& renderParams, const ShadowParameters& shadowParams, std::string hdrPath) : m_backend(backend), m_eventHandler(eventHandler), m_singleMode(singleMode), m_rParams(renderParams), m_sParams(shadowParams), m_showFPS(false), m_frameCount(0), m_hdrPath(hdrPath) {
	m_lastFPSComp = std::chrono::system_clock::now();
	backend->setRenderCallback(std::bind(&Graphene::Impl::render, this));
	backend->setExitCallback(std::bind(&Graphene::Impl::exit, this));
	backend->setAddVisCallback(std::bind(&Graphene::Impl::addVisualizer, this, std::placeholders::_1, std::placeholders::_2));
	backend->setDelVisCallback(std::bind(&Graphene::Impl::removeVisualizer, this, std::placeholders::_1));
	m_eventHandler->registerReceiver<void (Keys::Modifier)>("MODIFIER_PRESS",   "mainapp", std::bind(&Graphene::Impl::modifier, this, std::placeholders::_1, true));
	m_eventHandler->registerReceiver<void (Keys::Modifier)>("MODIFIER_RELEASE", "mainapp", std::bind(&Graphene::Impl::modifier, this, std::placeholders::_1, false));

#ifdef USE_SPACENAV
	m_spaceNav = std::make_shared<FW::Events::SpaceNav>();
#endif

	auto main = m_backend->getMainSettings();
    auto groupRendering = main->add<Section>("Rendering", "groupRendering");
	auto bg = groupRendering->add<Color>("Background: ", "background");
    bg->setValue(Eigen::Vector4f(1.f, 1.f, 1.f, 1.f));
    initRenderer();
}

Graphene::Impl::~Impl() {
}

void Graphene::Impl::initRenderer() {
    // init camera
	auto glSize = m_backend->getGLSize();
    auto model = harmont::camera_model::looking_at<harmont::orbit_camera_model>(vec3_t(0.0, -20.0, 0.0));
    m_camera = std::make_shared<harmont::camera>(model, glSize[0], glSize[1], 40.f, 0.01f, 200.f);

    // init renderer
    m_lightDir = Eigen::Vector3f(1.f, 1.f, 1.f).normalized();
    auto main    = m_backend->getMainSettings();
    Vector4f bg = main->get<Color>({"groupRendering", "background"})->value();
    harmont::deferred_renderer::render_parameters_t rParams {
        m_lightDir,
        bg.head(3),
        m_rParams.exposure,
        m_rParams.shadowBias,
        m_rParams.twoSided,
        m_hdrPath
    };
    harmont::deferred_renderer::shadow_parameters_t sParams {
        m_sParams.resolution,
        m_sParams.sampleCount
    };
    m_renderer = std::make_shared<harmont::deferred_renderer>(rParams, sParams, glSize[0], glSize[1]);

    // register relevant callbacks
	m_eventHandler->registerReceiver<void (int, int)>(
        "WINDOW_RESIZE",
        "mainapp",
        std::function<void (int, int)>([&] (int w, int h) {
            m_camera->reshape(w, h);
            m_renderer->reshape(m_camera);
        })
    );
	m_eventHandler->registerReceiver(
		"RIGHT_DRAG",
		"Camera",
		std::function<void (int,int,int,int)>(
			[&](int dx, int dy, int x, int y) {
				int m = 0;
				if (m_eventHandler->modifier()->alt() || m_eventHandler->modifier()->ctrl() && m_eventHandler->modifier()->shift()) return;
				if (m_eventHandler->modifier()->ctrl() )  {
                    m_camera->update(0.01f * vec3_t(-dx, -dy, 0.0), vec3_t::Zero());
                } else if (m_eventHandler->modifier()->shift()  )  {
                    m_camera->update(-1.f * vec3_t(0.0, 0.0, 0.3f * dy), vec3_t::Zero());
                } else {
                    m_camera->update(vec3_t::Zero(), 0.01f * vec3_t(dx, dy, 0.0));
                }
			}
		)
	);
	m_eventHandler->registerReceiver(
		"SCROLL",
		"Camera",
		std::function<void (int)>(
			[&](int d) {
                m_camera->update(1.f * vec3_t(0.0, 0.0, d), vec3_t::Zero());
			}
		)
	);

    initRenderProperties();
}

void Graphene::Impl::initRenderProperties() {
	auto main = m_backend->getMainSettings();

    auto groupRendering = main->get<Section>({"groupRendering"});

	auto  projection  = groupRendering->add<Choice>("Projection:");
	projection->add("perspective", "Perspective");
	projection->add("ortho", "Orthographic");
	projection->setCallback([&] (std::string mode) {
        m_camera->set_ortho(mode == "ortho");
    });

	auto bg = groupRendering->get<Color>({"background"});
    bg->setCallback([&] (Eigen::Vector4f color) { m_renderer->set_background_color(color.head(3)); });

    groupRendering->add<Button>("View Direction -> Light Direction", "setLightDir")->setCallback([&] () {
        m_lightDir = m_camera->forward().normalized();
    });
	auto exposure = groupRendering->add<Range>("Exposure", "exposure");
    exposure->setDigits(3).setMin(0.f).setMax(1.f).setValue(m_renderer->exposure());
    exposure->setCallback([&] (float e) { m_renderer->set_exposure(e); });

	auto bias = groupRendering->add<Range>("Shadow Bias", "bias");
    bias->setDigits(3).setMin(0.f).setMax(0.01f).setValue(m_renderer->shadow_bias());
    bias->setCallback([&] (float b) { m_renderer->set_shadow_bias(b); });

	auto groupSSDO = groupRendering->add<Section>("SSDO", "groupSSDO");
	auto ssdoRadius = groupSSDO->add<Range>("Radius", "radius");
    ssdoRadius->setDigits(2).setMin(0.f).setMax(3.f).setValue(m_renderer->ssdo_radius());
    ssdoRadius->setCallback([&] (float r) { m_renderer->set_ssdo_radius(r); });
	auto ssdoExponent = groupSSDO->add<Range>("Exponent", "exponent");
    ssdoExponent->setDigits(2).setMin(0.f).setMax(5.f).setValue(m_renderer->ssdo_exponent());
    ssdoExponent->setCallback([&] (float e) { m_renderer->set_ssdo_exponent(e); });
	auto ssdoReflAlbedo = groupSSDO->add<Range>("Reflective Albedo", "reflective_albedo");
    ssdoReflAlbedo->setDigits(2).setMin(0.f).setMax(5.f).setValue(m_renderer->ssdo_reflective_albedo());
    ssdoReflAlbedo->setCallback([&] (float a) { m_renderer->set_ssdo_reflective_albedo(a); });

	auto groupSplats = groupRendering->add<Section>("Splat Rendering", "groupSplats");
	auto pointSize = groupSplats->add<Range>("Splat Size", "splat_size");
    pointSize->setDigits(2).setMin(0.f).setMax(5.f).setValue(m_renderer->point_size());
    pointSize->setCallback([&] (float s) { m_renderer->set_point_size(s); });
}

/*
void Graphene::Impl::initTransforms() {
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
*/

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
	VisualizerHandle::Ptr   fwHandle(new VisualizerHandle(visName, m_eventHandler, m_camera));
	auto guiHandle = m_backend->addVisualizer(visName);
	if (!guiHandle) return;
	vis->setHandles(fwHandle, guiHandle);
    vis->setRenderer(m_renderer);
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
#ifdef USE_SPACENAV
	auto motion = m_spaceNav->motion();
	m_camera->update(vec3_t(-motion[0], motion[2], -motion[1]), 0.1f * vec3_t(motion[5], motion[3], 0.f));
#endif
	
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

    // wait for pending tasks
	if (m_singleMode) {
		for (const auto& vis : m_visualizer) {
			vis.second->waitForTasks();
		}
	} else {
		auto  names = m_backend->getActiveVisualizerNames();
		for (const auto& name : names) {
			m_visualizer[name]->waitForTasks();
		}
	}

    m_renderer->set_light_dir(m_lightDir);
    m_renderer->render(m_camera);
}

void Graphene::Impl::modifier(Keys::Modifier mod, bool down) {
	switch (mod) {
		case Keys::SHIFT: m_eventHandler->modifier()->shift() = down; break;
		case Keys::CTRL:  m_eventHandler->modifier()->ctrl()  = down; break;
		case Keys::ALT:   m_eventHandler->modifier()->alt()   = down; break;
		default:          m_eventHandler->modifier()->altgr() = down;
	}
}

//void Graphene::Impl::setCameraControl(std::string control) {
	//m_camera->setControl(m_camControls[control]);
//}

//void Graphene::Impl::setOrtho(bool ortho) {
	//m_camera->setOrtho(ortho);
//}

} // FW
