/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef FWEVENTHANDLER_H_
#define FWEVENTHANDLER_H_

/**
 *
 *  @internal @file EventHandler.h
 *
 *  @brief Core event management system.
 *
 *  This file specifies the Signal and EventHandler classes
 *  that implement the framework's event management system.
 *
 */

#include <include/common.h>

#include <Generic/Command.h>
using Generic::Command;
using Generic::BaseCommand;

#include "Modifier.h"



namespace FW {
namespace Events {

/** @internal Event type descriptor (typedef'd std::string) */
typedef std::string EventType;


/** 
 *  @internal Signal
 *  
 *  @brief Generic signal type
 */
class Signal {
	public:
		typedef std::shared_ptr<Signal>  Ptr;
		typedef boost::any               Param;
		typedef std::vector<Param>       Params;
		class Impl;

	public:
		/**
		 *  @brief Constructor.
		 *  @param type Type of event as std::string
		 *  @see Events::Handle for possible values of type
		 */
		Signal(std::string type);

		/**
		 *  @brief Add parameter to signal.
		 *  @param param Parameter value of any type (i.e. type is irrelevant since any type is implicitly castable to boost::any).
		 */
		void addParam(Param param);

		/**
		 *  @brief Get parameters.
		 *  @return Returns a std::vector of parameters given as boost::any values.
		 */
		const Params& getParams() const;

		/**
		 *  @brief Get type of event.
		 *  @return Returns the type of event this signal has been created for.
		 */
		std::string getType() const;

	private:
		std::shared_ptr<Impl> m_impl;
};


/** 
 *  @internal EventHandler
 *  
 *  @brief Management class for events/signals
 *
 *  The backend uses this class to notify the framework about events.
 *  Any receivers registered using registerReceiver
 *  are then again notified using the Signal mechanism.
 *
 *  Visualizers can access this management class through the Handle interface class.
 *
 *  @see Handle for possible events
 *
 */
class EventHandler {
	public:
		typedef std::shared_ptr<EventHandler>  Ptr;
		class Impl;

	public:
		/**
		 *  Constructor.
		 */
		EventHandler();

		/**
		 *  Registers a callback as receiver for given event type
		 *
		 *  @tparam Sig Function signature of the given callback.
		 *  @param  eventType Type of event to connect to.
		 *  @param  id Unique ID of receiver (usually the Visualizer ID).
		 *  @param  receiver Callback function to call on notify.
		 *  @see unregisterReceiver(std::string eventType, std::string id) and unregisterReceiver(std::string id)
		 */
		template <class Sig>
		void registerReceiver(std::string eventType, std::string id, std::function<Sig> receiver);

		/**
		 *  Disconnects callbacks of given event type for given receiver.
		 *  
		 *  @param eventType Type of events to disconnect.
		 *  @param id Unique ID of receiver to disconnect.
		 *  @see unregisterReceiver(std::string id)
		 */
		void unregisterReceiver(std::string eventType, std::string id);

		/**
		 *  Disconnects all callbacks for given receiver.
		 *  
		 *  @param id Unique ID of receiver to disconnect.
		 *  @see unregisterReceiver(std::string eventType, std::string id)
		 */
		void unregisterReceiver(std::string id);

		/**
		 *  Notify the framework that an event occured.
		 *
		 *  Calling this method with a signal results in all receivers
		 *  connected to the corresponding event type to be notified
		 *  about the event.
		 *
		 *  @param signal A Signal specifying the event type and its parameters
		 */
		void notify(Signal signal);

		/**
		 *  Get access handle to modifier states.
		 *  
		 *  @returns Modifier handle.
		 */
		Modifier::Ptr modifier();

		// blocks are ignored by general events and window events (pre/post/resize)
		/**
		 *  Unblock all signals.
		 *
		 *  If any signals have been blocked, unblock again.
		 *
		 *  @see allow(std::string id), allowAllBut(std::string id), blockAll(), block(std::string id), blockAllBut(std::string id)
		 */
		void allowAll();

		/**
		 *  Block all signals except PRE_EVENT, POST_EVENT and WINDOW_RESIZE.
		 *
		 *  This function blocks emission of all signals except for the essential signal types PRE_EVENT, POST_EVENT and WINDOW_RESIZE.
		 *
		 *  @see allowAll(), allow(std::string id), allowAllBut(std::string id), block(std::string id), blockAllBut(std::string id)
		 */
		void blockAll();

		/**
		 *  Unblock signals for specified receiver.
		 *
		 *  If any signals have been blocked for the given receiver, unblock again.
		 *
		 *  @see allowAll(), allowAllBut(std::string id), blockAll(), block(std::string id), blockAllBut(std::string id)
		 */
		void allow(std::string id);

		/**
		 *  Block all signals for given receiver except PRE_EVENT, POST_EVENT and WINDOW_RESIZE.
		 *
		 *  This function blocks emission of all signals for the given receiver except for the essential signal types PRE_EVENT, POST_EVENT and WINDOW_RESIZE.
		 *
		 *  @see allowAll(), allow(std::string id), allowAllBut(std::string id), blockAll(), blockAllBut(std::string id)
		 */
		void block(std::string id);

		/**
		 *  Block all signals except for given receiver.
		 *
		 *  This function is equivalent to calling
		 *      blockAll();
		 *      allow(id);
		 *
		 *  @see allowAll(), allow(std::string id), allowAllBut(std::string id), blockAll(), block(std::string id)
		 */
		void blockAllBut(std::string id);

		/**
		 *  Allow all signals except for given receiver.
		 *
		 *  This function is equivalent to calling
		 *      allowAll();
		 *      block(id);
		 *
		 *  @see allowAll(), allow(std::string id), blockAll(), block(std::string id), blockAllBut(std::string id)
		 */
		void allowAllBut(std::string id);

	protected:
		std::shared_ptr<Impl> m_impl;
};


} // Events
} // FW


#endif /* FWEVENTHANDLER_H_ */
