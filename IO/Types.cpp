#include "Types.h"

namespace IO {

Flags::PrintOptions Flags::CLEAR = 1<<0;
Flags::PrintOptions Flags::FLUSH = 1<<1;
Flags::PrintOptions Flags::END = 1<<2;
Flags::PrintOptions Flags::UPDATE = Flags::CLEAR | Flags::FLUSH;
Flags::PrintOptions Flags::LASTUPDATE = Flags::END | Flags::CLEAR | Flags::FLUSH;

#ifdef USE_BASH_COLORS
std::string Types::color(Color col) {
	switch (col) {
		case RED:    return std::string("\033[1;31m");
		case GREEN:  return std::string("\033[1;32m");
		case YELLOW: return std::string("\033[1;33m");
		case BLUE:   return std::string("\033[1;34m");
		default: return std::string("\e[0m");
	}
}
#endif

} // IO
