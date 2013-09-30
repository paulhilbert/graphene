#include "Log.h"

namespace IO {

void Log::info(std::string s, Flags::PrintOptions options, bool prefix) {
#ifdef USE_BASH_COLORS
	std::string msg = (prefix) ? Types::color(Types::GREEN) + "[I]  " + Types::color() + s : "     " + s;
#else
	std::string msg = (prefix) ? "[I]  " + s : "     " + s;
#endif
	Console::printOut(msg, options);
}

void Log::warn(std::string s, Flags::PrintOptions options, bool prefix) {
#ifdef USE_BASH_COLORS
	std::string msg = (prefix) ? Types::color(Types::YELLOW) + "[W]  " + Types::color() + s : "     " + s;
#else
	std::string msg = (prefix) ? "[W]  " + s : "     " + s;
#endif
	Console::printOut(msg, options);
}

void Log::error(std::string s, Flags::PrintOptions options, bool prefix) {
#ifdef USE_BASH_COLORS
	std::string msg = (prefix) ? Types::color(Types::RED) + "[E]  " + Types::color() + s : "     " + s;
#else
	std::string msg = (prefix) ? "[E]  " + s : "     " + s;
#endif
	Console::printOut(msg, options);
}

void Log::verbose(std::string s, Flags::PrintOptions options, bool prefix) {
#ifdef USE_BASH_COLORS
	std::string msg = (prefix) ? Types::color(Types::BLUE) + "[V]  " + Types::color() + s : "     " + s;
#else
	std::string msg = (prefix) ? "[V]  " + s : "     " + s;
#endif
	Console::printOut(msg, options);
}

} // IO
