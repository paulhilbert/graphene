/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include <include/config.h>
#include <include/common.h>
#include <include/ogl.h>

// boost
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/extension/factory.hpp>
#include <boost/extension/shared_library.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ext = boost::extensions;
using boost::regex;
using boost::smatch;
using boost::regex_match;

// graphene
#include <FW/FWGraphene.h>
using FW::Graphene;

#include <FW/Events/EventsEventHandler.h>
#include <GUI/GUIBackend.h>

GUI::Backend::Ptr getBackend(std::string path);
void substituteHome(std::string& path);

int main( int argc, char *argv[] ) {
	std::string  visPath;
	std::string  visExt;
	std::string  backendPath;
	std::string  hdrPath;
    std::string  cameraModel;
	std::string  single;
	std::string  title;
	std::string  stylesheet;
	int          fps;
	int          wndWidth, wndHeight;
	//bool         noEffects;
	bool         verbose;
	bool         singleMode;
    Graphene::RenderParameters rParams;
    Graphene::ShadowParameters sParams;

	GUI::Backend::WindowParams wndParams;

	po::options_description desc("Graphene command line options");
	desc.add_options()
		("help,h",  "Help message")
		("version,v",  "Print version")
		("visPath", po::value<std::string>(&visPath) ->required(), "Path to visualizer shared libraries")
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
		("visExt",  po::value<std::string>(&visExt)  ->default_value(".dll"), "File extension of visualizer shared libraries")
		("backend", po::value<std::string>(&backendPath) ->default_value("libGrapheneQt5.dll"), "Path to backend library")
#else
		("visExt",  po::value<std::string>(&visExt)  ->default_value(".so"), "File extension of visualizer shared libraries")
		("backend", po::value<std::string>(&backendPath) ->default_value(PREFIX"/lib/libGrapheneQt5.so"), "Path to backend library")
#endif
		("hdrPath",    po::value<std::string>(&hdrPath)->default_value(""), "Path to HDR environment maps")
		("camera",     po::value<std::string>(&cameraModel)->default_value("orbit"), "Model for camera control (either \"orbit\" (default) or \"fly\")")
		("single",     po::value<std::string>(&single) ->default_value(""), "Use the given name as single mode visualizer")
		("title",      po::value<std::string>(&title) ->default_value("graphene"), "Window title")
		("stylesheet", po::value<std::string>(&stylesheet) ->default_value(""), "Stylesheet for the UI")
		("exposure", po::value<float>(&rParams.exposure)->default_value(0.7), "Initial exposure")
		("shadow_bias", po::value<float>(&rParams.shadowBias)->default_value(0.003), "Initial shadow bias")
		("shadow_res", po::value<uint32_t>(&sParams.resolution)->default_value(2048), "Shadow map resolution")
		("shadow_samples", po::value<uint32_t>(&sParams.sampleCount)->default_value(32), "Number of shadow samples")
		("width",      po::value<int>(&wndWidth)  ->default_value(1024), "Path to backend library")
		("height",     po::value<int>(&wndHeight) ->default_value(576), "Path to backend library")
		("fps",        po::value<int>(&fps) ->default_value(60), "Frames Per Second")
		("logX",       po::value<int>(&wndParams.logX)->default_value(-1), "Log window position x")
		("logY",       po::value<int>(&wndParams.logY)->default_value(-1), "Log window position y")
		("logWidth",   po::value<int>(&wndParams.logWidth)->default_value(-1), "Log window width")
		("logHeight",  po::value<int>(&wndParams.logHeight)->default_value(-1), "Log window height")
		("propX",      po::value<int>(&wndParams.propX)->default_value(-1), "Property window position x")
		("propY",      po::value<int>(&wndParams.propY)->default_value(-1), "Property window position y")
		("propWidth",  po::value<int>(&wndParams.propWidth)->default_value(-1), "Property window width")
		("propHeight", po::value<int>(&wndParams.propHeight)->default_value(-1), "Property window height")
		("maximized", "Show main window maximized")
		("single_sided", "Do not render two sided primitives.")
		("verbose", "Output additional debug messages")
	;

	// Check for required options.
	po::variables_map vm;
	bool optionsException = false;
	try {
		po::store(po::parse_command_line(argc, argv, desc), vm);
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
		fs::path cfgFile("graphene.conf");
#else
		fs::path cfgFile(std::string(getenv("HOME"))+"/.graphene.conf");
#endif
		std::ifstream in;
		if (fs::exists(cfgFile)) {
			in.open(cfgFile.string().c_str());
			po::store(po::parse_config_file(in, desc), vm);
			in.close();
		} else {
			std::cout << "config file does not exist" << "\n";
		}
		po::notify(vm);
	} catch (std::exception& e) {
		if (!vm.count("help")) {
			std::cout << e.what() << "\n";
		}
		optionsException = true;
	}
	if (optionsException || vm.count("help")) {
		std::cout << desc << "\n";
		return optionsException ? 1 : 0;
	}
	if (vm.count("version")) {
		std::cout << "graphene version: " << VERSION_MAJOR << "." << VERSION_MINOR << "\n";
		return 0;
	}
	wndParams.maximized = vm.count("maximized") > 0;
	verbose = vm.count("verbose") > 0;
	singleMode = single != "";

	// sanitize paths
#if !(defined(WIN32) || defined(_WIN32) || defined(__WIN32))
	substituteHome(visPath);
	substituteHome(backendPath);
#endif

    if (cameraModel != "orbit" && cameraModel != "fly") {
        std::cout << "Camera model is invalid. Set to \"orbit\"." << "\n";
        cameraModel = "orbit";
    }

	FW::Events::EventHandler::Ptr eventHandler(new FW::Events::EventHandler());

	GUI::Backend::Ptr backend = getBackend(backendPath);
	if (!backend) return 1;
	backend->init(argc, argv, eventHandler, wndParams, singleMode, verbose);
	backend->setWindowTitle(title);
	backend->setWindowSize(wndWidth, wndHeight);
	if (stylesheet != "") backend->setStylesheet(stylesheet);

    rParams.twoSided = !vm.count("single_sided");
	Graphene graphene(backend, eventHandler, singleMode, rParams, sParams, hdrPath, cameraModel);

	if (singleMode) {
		fs::path p(visPath);
		p = p / ("vis"+single+visExt);
		if (fs::is_directory(p)) {
			backend->getLog()->error("Input factory is a directory: " + p.string());
			return graphene.run(fps);
		}
		if (!fs::exists(p)) {
			backend->getLog()->error("Input factory does not exist: " + p.string());
			return graphene.run(fps);
		}
		ext::shared_library lib(p.string());
		if (!lib.open()) {
			backend->getLog()->error("Library failed to open: " + p.string());
			return graphene.run(fps);
		}
		std::function<FW::Factory* ()> retrFactory(lib.get<FW::Factory*>("getFactory"));
		if (!retrFactory) {
			backend->getLog()->error("Function \"getFactory\" not found in \""+p.string()+"\". Try adding VIS_DLL_EXPORT(VIS) macro.");
			return graphene.run(fps);
		}
		backend->getLog()->info("Visualizer type \"" + single + "\" initialized");
		FW::Factory* factoryPtr = retrFactory();
		std::shared_ptr<FW::Factory> factory(factoryPtr);
		graphene.addFactory(single, factory);
	} else {
		fs::directory_iterator dirIt(visPath), endIt;
		for (; dirIt != endIt; ++dirIt) {
			fs::path p = dirIt->path();
			if (fs::is_directory(p)) continue;
			smatch what;
			regex pattern("vis(\\w+)"+visExt);
			std::string filename = p.filename().string();
			if (!regex_match(filename, what, pattern)) continue;
			std::string name = what[1];
			ext::shared_library lib(p.string());
			if (!lib.open()) {
				backend->getLog()->warn("Library failed to open: " + p.string());
				continue;
			}
			std::function<FW::Factory* ()> retrFactory(lib.get<FW::Factory*>("getFactory"));
			if (!retrFactory) {
				backend->getLog()->warn("Function \"getFactory\" not found in \""+p.string()+"\". Try adding VIS_DLL_EXPORT(VIS) macro.");
				continue;
			}
			backend->getLog()->info("Adding visualizer type: " + name);
			FW::Factory* factoryPtr = retrFactory();
			std::shared_ptr<FW::Factory> factory(factoryPtr);
			graphene.addFactory(name, factory);
		}
	}

	return graphene.run(fps);
}

GUI::Backend::Ptr getBackend(std::string path) {
	ext::shared_library lib(path);
	if (!lib.open()) {
		std::cout << "Error: Could not open backend! Aborting." << "\n";
		return GUI::Backend::Ptr();
	}
	std::function<GUI::Backend* (void)> f(lib.get<GUI::Backend*>(std::string("getBackend")));
	if (!f) {
		std::cout << "Error: Function getBackend not found! Aborting." << "\n";
		return GUI::Backend::Ptr();
	}
	GUI::Backend::Ptr backend(f());
	return backend;
}

inline void substituteHome(std::string& path) {
	regex pattern("~");
	std::ostringstream t(std::ios::out | std::ios::binary);
	std::ostream_iterator<char, char> oi(t);
	boost::regex_replace(oi, path.begin(), path.end(),	pattern, std::string(getenv("HOME")));
	path = t.str();
}
