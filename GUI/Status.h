#ifndef GUISTATUS_H_
#define GUISTATUS_H_

/**
 *  @file Status.h
 *
 *  Defines access handle to status bar.
 */

#include <include/common.h>

namespace GUI {

/**
 *  Access handle to status bar.
 */
class Status {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Status> Ptr;
		/** Weak pointer to this class */
		typedef std::weak_ptr<Status>   WPtr;

	public:
		/** Constructor */
		Status();

		/** Destructor */
		virtual ~Status();

		/**
		 *  Set text to display.
		 *
		 *  @param text Message to display.
		 */
		virtual void set(const std::string& text) = 0;

		/**
		 *  Get currently displayed text.
		 *
		 *  @return Current text in status bar.
		 */
		virtual std::string  get() const = 0;

		/** Clear message text. */
		virtual void clear() = 0;
};

} // GUI

#endif /* GUISTATUS_H_ */
