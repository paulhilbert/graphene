#ifndef CONSOLE_H_
#define CONSOLE_H_

#include <iosfwd>

#include "Types.h"

namespace IO {

struct Console {
	static std::string clearLine() { return std::string("\33[2K\r"); }

	static void printOut(std::string s, Flags::PrintOptions options=Flags::END, std::string prefix="");
	static void printErr(std::string s, Flags::PrintOptions options=Flags::END, std::string prefix="");
	static void printLn(std::ostream& out, std::string s, Flags::PrintOptions options=Flags::END, std::string prefix="");
};

} // IO

#endif /* CONSOLE_H_ */
