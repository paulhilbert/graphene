#include <iostream> // for graphics

/* boost */
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

/* commons */
#include <Testing/Profiling.h>
#include <IO/Log.h>
using namespace IO;


int main(int argc, char* argv[]) {
	std::string training, testing;
	std::string dbFile;

	po::options_description desc("General command line options");
	desc.add_options()
		("help,h", "Help message")
		("integer,i", po::value<int>(&integer), "integer param")
		("str,s", po::value<std::string>(&str), "string param")
		("bool", "optional bool")
	;

	po::variables_map vm;
	bool optionsException = false;
	try {
		po::store(po::parse_command_line(argc, argv, desc), vm);
		fs::path cfgFile("conf/"+ fs::path(std::string(argv[0])).filename() + ".conf");
		std::ifstream in;
		if (fs::exists(cfgFile)) {
			in.open(cfgFile.file_string().c_str());
			po::store(po::parse_config_file(in, desc), vm);
			in.close();
		}
		po::notify(vm);
	} catch (exception& e) {
		if (!vm.count("help")) {
			Log::error(e.what());
		}
		optionsException = true;
	}
	
	if (optionsException || vm.count("help")) {
		std::cout << desc << "\n";
		return 1;
	}
	

	
}
