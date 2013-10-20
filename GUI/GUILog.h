#ifndef GUILOG_H_
#define GUILOG_H_

#include <include/common.h>

namespace GUI {

class Log {
	public:
		typedef std::shared_ptr<Log> Ptr;
		typedef std::weak_ptr<Log> WPtr;

	public:
		Log(bool verbose);
		virtual ~Log();

		virtual void info(std::string text)  = 0;
		virtual void warn(std::string text)  = 0;
		virtual void error(std::string text) = 0;
		virtual void verbose(std::string text) = 0;

		virtual void clear() = 0;

		virtual void fail(std::string text) = 0;

	protected:
		bool m_verbose;
};

} // GUI

#endif /* GUILOG_H_ */
