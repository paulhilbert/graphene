/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5LOG_H_
#define QT5LOG_H_

#include <GUI/GUILog.h>
#include "Qt5LogDialog.h"

namespace GUI {

class Qt5Log : public Log {
	public:
		typedef std::shared_ptr<Qt5Log> Ptr;
		typedef std::weak_ptr<Qt5Log> WPtr;

	public:
		Qt5Log(Qt5LogDialog* log, bool verbose);
		virtual ~Qt5Log();

		void info(std::string text);
		void warn(std::string text);
		void error(std::string text);
		void verbose(std::string text);
		void clear();

		void fail(std::string text);

	protected:
		Qt5LogDialog*  m_log;
};


} // GUI

#endif /* QT5LOG_H_ */
