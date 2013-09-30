#include "Console.h"

#include <iostream>

namespace IO {

void Console::printOut(std::string s, Flags::PrintOptions options, std::string prefix) {
	printLn(std::cout, s, options, prefix);
}

void Console::printErr(std::string s, Flags::PrintOptions options, std::string prefix) {
	printLn(std::cerr, s, options, prefix);
}

void Console::printLn(std::ostream& out, std::string s, Flags::PrintOptions options, std::string prefix) {
	if (options & Flags::CLEAR) out << Console::clearLine();
	out << prefix << s;
	if (options & Flags::FLUSH) out << std::flush;
	if (options & Flags::END) out << "\n";
}

} // IO
