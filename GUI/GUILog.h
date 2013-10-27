/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef GUILOG_H_
#define GUILOG_H_

/**
 *  @file GUILog.h
 *
 *  @brief Defines access class to log facility.
 */

#include <include/common.h>

namespace GUI {

/**
 *  Access class to log facility.
 */
class Log {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Log> Ptr;
		/** Weak pointer to this class */
		typedef std::weak_ptr<Log> WPtr;

	public:
		/**
		 *  Constructor
		 *
		 *  @param verbose Print verbose messages
		 */
		Log(bool verbose);

		/** Destructor */
		virtual ~Log();

		/**
		 *  Print message of importance level INFO
		 *
		 *  @param text Message to print
		 */
		virtual void info(std::string text)  = 0;

		/**
		 *  Print message of importance level WARNING
		 *
		 *  @param text Message to print
		 */
		virtual void warn(std::string text)  = 0;

		/**
		 *  Print message of importance level ERROR
		 *
		 *  @param text Message to print
		 */
		virtual void error(std::string text) = 0;

		/**
		 *  Print message of importance level VERBOSE
		 *
		 *  This function does nothing if Log was constructed with parameter verbose == false
		 *
		 *  @param text Message to print
		 */
		virtual void verbose(std::string text) = 0;

		/** Clear messages */
		virtual void clear() = 0;

		/** 
		 *  Inform user about critical errors.
		 *
		 *  Backend implementations should inform the user with a modal dialog
		 *  so the execution does not continue (probably in a broken state) without
		 *  the user knowing about the supplied message.
		 *
		 *  @param text Message to print.
		 */
		virtual void fail(std::string text) = 0;

	protected:
		bool m_verbose;
};

} // GUI

#endif /* GUILOG_H_ */
