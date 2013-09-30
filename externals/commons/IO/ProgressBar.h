#ifndef IOPROGRESSBAR_H_
#define IOPROGRESSBAR_H_

#include <string>
#include <functional>

#include "Log.h"

namespace IO {

class ProgressBar {
	public:
		typedef std::function<void (std::string, Flags::PrintOptions)> PrintFunc;
	public:
		ProgressBar(int steps = 1, int width = 20, bool printPercent = true);
		std::string poll(float progress);
		std::string poll(unsigned int done, unsigned int todo);

		void print(unsigned int done, unsigned int todo, std::string prefix = "", std::string suffix = "", const PrintFunc& printFunc = std::bind(&Log::info, std::placeholders::_1, std::placeholders::_2, true));
		void printVerbose(unsigned int done, unsigned int todo, std::string prefix = "", std::string suffix = "");

	protected:
		int  m_steps;
		int  m_width;
		bool m_printPercent;
};

} // IO

#endif /* IOPROGRESSBAR_H_ */
