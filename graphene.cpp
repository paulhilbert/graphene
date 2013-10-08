// sys
#include <iostream> // for graphics
#include <fstream>
#include <memory>

// gl

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
#include <windows.h>
#endif

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>


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

#include <FW/Events/EventHandler.h>
#include <GUI/Backend.h>

GUI::Backend::Ptr getBackend(std::string path);

int main( int argc, char *argv[] ) {
	std::string  visPath;
	std::string  visExt;
	std::string  backendPath;
	int          fps;
	int          wndWidth, wndHeight;
	bool         verbose;

	po::options_description desc("Graphene command line options");
	desc.add_options()
		("help,h",  "Help message")
		("visPath", po::value<std::string>(&visPath) ->required(), "Path to visualizer shared libraries")
		("visExt",  po::value<std::string>(&visExt)  ->required(), "File extension of visualizer shared libraries")
		("backend", po::value<std::string>(&backendPath) ->required(), "Path to backend library")
		("width",   po::value<int>(&wndWidth)  ->default_value(1024), "Path to backend library")
		("height",  po::value<int>(&wndHeight) ->default_value(576), "Path to backend library")
		("fps",     po::value<int>(&fps) ->default_value(60), "Frames Per Second")
		("verbose", "Output additional debug messages")
	;

	// Check for required options.
	po::variables_map vm;
	bool optionsException = false;
	try {
		po::store(po::parse_command_line(argc, argv, desc), vm);
		fs::path cfgFile("graphene.conf");
		std::ifstream in;
		if (fs::exists(cfgFile)) {
			in.open(cfgFile.string().c_str());
			po::store(po::parse_config_file(in, desc), vm);
			in.close();
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
		return 1;
	}
	verbose = vm.count("verbose") > 0;

	FW::Events::EventHandler::Ptr eventHandler(new FW::Events::EventHandler());

	GUI::Backend::Ptr backend = getBackend(backendPath);
	if (!backend) return 1;
	backend->init(argc, argv, eventHandler, verbose);
	backend->setWindowTitle("graphene");
	backend->setWindowSize(wndWidth, wndHeight);

	Graphene graphene(backend, eventHandler);

	fs::directory_iterator dirIt(visPath), endIt;
	regex pattern("vis(\\w+)"+visExt);
	for (; dirIt != endIt; ++dirIt) {
		fs::path p = dirIt->path();
		if (fs::is_directory(p)) continue;
		smatch what;
		if (!regex_match(p.filename().string(), what, pattern)) continue;
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
