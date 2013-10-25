#ifndef PROPERTYNOTIFY_H_
#define PROPERTYNOTIFY_H_

/**
 *  @file Notify.h
 *
 *  Defines base notification class for all properties.
 */

#include <include/common.h>

namespace GUI {
namespace Property {

/**
 *  Property base type that provides means to connect callback functions
 *  to change of state events of derived classes
 *
 *  Property classes derived from this base class have a setCallback(func)
 *  method the user can use to define a callback function in case the state
 *  of the corresponding property widget changes.
 *
 *  @tparam Sig Function signature of the callback function.
 */
template <class Sig>
class Notify {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Notify>      Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Notify>        WPtr;

		/** Callback function to trigger on changes */
		typedef std::function<Sig>  Callback;

	public:
		/**
		 *  @internal Notify()
		 *
		 *  @brief Constructor
		 */
		Notify();

		/**
		 *  @internal ~Notify()
		 *
		 *  @brief Destructor
		 */
		virtual ~Notify();

		/**
		 *  Sets function to call on state change.
		 *
		 *  @param onChange Callback function to call.
		 */
		void setCallback(Callback onChange);

		/**
		 *  Resets callback function to nullptr.
		 */
		void unsetCallback();

	protected:
		void notify() const;
		template <class Arg0>
		void notify(Arg0 arg0) const;
		template <class Arg0, class Arg1>
		void notify(Arg0 arg0, Arg1 arg1) const;

	protected:
		Callback m_onChange;
};

#include "Notify.inl"

} // Property
} // GUI

#endif /* PROPERTYNOTIFY_H_ */
