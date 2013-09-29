#ifndef LOG_H_
#define LOG_H_

#include "Console.h"

namespace IO {

class Log {
	public:
		static void info(std::string s, Flags::PrintOptions options=Flags::END, bool prefix=true);
		static void warn(std::string s, Flags::PrintOptions options=Flags::END, bool prefix=true);
		static void error(std::string s, Flags::PrintOptions options=Flags::END, bool prefix=true);
		static void verbose(std::string s, Flags::PrintOptions options=Flags::END, bool prefix=true);
};

}

#endif /* LOG_H_ */
