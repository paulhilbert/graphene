#ifndef EVENTSVISUALIZERHANDLE_H_
#define EVENTSVISUALIZERHANDLE_H_

/**
 *  @file FW/Events/Handle.h
 *
 *  Specifies access class to event management system.
 *
 */

#include <include/common.h>
#include "EventHandler.h"


namespace FW {

class VisualizerHandle;

namespace Events {

/**
 *  @brief Access handle to event management system.
 *
 *  This class is internally constructed and supplied to
 *  visualizers in order to connect events (primarily input and window events)
 *  to local callback functions.
 *
 *  Example usage in visualizer functions:
 *
 *      void CustomVisualizer::init() {
 *          auto onClick = std::bind(&CustomVisualizer::doSthOnClick, std::placeholders::_1, std::placeholders::_2);
 *          fw()->events()->connect<void (int, int)>("LEFT_CLICK", onClick);
 *      }
 *      ...
 *      void CustomVisualizer::doSthOnClick(int x, int y) {
 *          std::cout << "User clicked at position: " << x << ", " << y << "\n";
 *      }
 *
 *  This example would call doSthOnClick everytime the user left-clicks inside the OpenGL area.
 *  The user has to supply the correct function signature and signal type.
 *
 *  Available signals are:
 *
 *  Signal Type  | Function Signature | Parameter Description
 *  ------------- | ------------- | -------------
 *  "LEFT_PRESS"          | void (int x, int y)                      | Mouse pos as (x, y) coords.
 *  "RIGHT_PRESS"         | void (int x, int y)                      | Mouse pos as (x, y) coords.
 *  "MIDDLE_PRESS"        | void (int x, int y)                      | Mouse pos as (x, y) coords.
 *  "LEFT_RELEASE"        | void (int x, int y)                      | Mouse pos as (x, y) coords.
 *  "RIGHT_RELEASE"       | void (int x, int y)                      | Mouse pos as (x, y) coords.
 *  "MIDDLE_RELEASE"      | void (int x, int y)                      | Mouse pos as (x, y) coords.
 *  "LEFT_CLICK"          | void (int x, int y)                      | Mouse pos as (x, y) coords.
 *  "RIGHT_CLICK"         | void (int x, int y)                      | Mouse pos as (x, y) coords.
 *  "MIDDLE_CLICK"        | void (int x, int y)                      | Mouse pos as (x, y) coords.
 *  "MOVE"                | void (int dx, int dy, int x, int y)      | Mouse change in pos as (dx, dy). Mouse pos as (x, y) coords.
 *  "LEFT_DRAG"           | void (int dx, int dy, int x, int y)      | Mouse change in pos as (dx, dy). Mouse pos as (x, y) coords.
 *  "RIGHT_DRAG"          | void (int dx, int dy, int x, int y)      | Mouse change in pos as (dx, dy). Mouse pos as (x, y) coords.
 *  "MIDDLE_DRAG"         | void (int dx, int dy, int x, int y)      | Mouse change in pos as (dx, dy). Mouse pos as (x, y) coords.
 *  "LEFT_DRAG_START"     | void (int dx, int dy)                    | Mouse pos as (x, y) coords.
 *  "RIGHT_DRAG_START"    | void (int dx, int dy)                    | Mouse pos as (x, y) coords.
 *  "MIDDLE_DRAG_START"   | void (int dx, int dy)                    | Mouse pos as (x, y) coords.
 *  "LEFT_DRAG_STOP"      | void (int dx, int dy)                    | Mouse pos as (x, y) coords.
 *  "RIGHT_DRAG_STOP"     | void (int dx, int dy)                    | Mouse pos as (x, y) coords.
 *  "MIDDLE_DRAG_STOP"    | void (int dx, int dy)                    | Mouse pos as (x, y) coords.
 *  "SCROLL"              | void (int delta)                         | Change of scroll wheel as signed integer.
 *  "MODIFIER_PRESS"      | void (FW::Events::Keys::Modifier mod)    | Modifier key pressed.
 *  "MODIFIER_RELEASE"    | void (FW::Events::Keys::Modifier mod)    | Modifier key released.
 *  "SPECIAL_KEY_PRESS"   | void (FW::Events::Keys::Special special) | Special key pressed.
 *  "SPECIAL_KEY_RELEASE" | void (FW::Events::Keys::Special special) | Special key released.
 *  "CHAR_KEY_PRESS"      | void (std::string char)                  | Character key pressed.
 *  "CHAR_KEY_RELEASE"    | void (std::string char)                  | Character key released.
 *  "WINDOW_RESIZE"       | void (int width, int height)             | New window size as (w, h).
 *  "PRE_EVENT"           | void (std::string type)                  | Type of signal to be emitted after this.
 *  "POST_EVENT"          | void (std::string type)                  | Type of signal emitted before this.
 *
 *  For other usage examples see the Event example visualizer in the visualizer repository.
 *
 */
class Handle {
	public:
		typedef std::shared_ptr<Handle> Ptr;
		typedef std::weak_ptr<Handle>   WPtr;
		friend class ::FW::VisualizerHandle;

	protected:
		Handle(std::string id, EventHandler::Ptr eventHandler);

	public:
		/**
		 *  Destructor.
		 */
		virtual ~Handle();

		/**
		 *  Connect callback to signal.
		 *  @tparam Sig Function signature of receiver function. See table below for possible options.
		 *  @param type Type of event as std::string. See table below for possible options.
		 *  @param receiver Callback function to call on signal emission.
		 *  @see disconnect(EventType type)
		 */
		template <class Sig>
		void connect(std::string type, std::function<Sig> receiver);

		/**
		 * Disconnect callback(s).
		 * @param type Type of event to disconnect. Defaults to disconnect all signals. This function is automatically called on Visualizer destruction.
		 * @see connect(EventType type, std::function<Sig> receiver)
		 */
		void disconnect(std::string type = "__all__");

	protected:
		std::string       m_id;
		EventHandler::Ptr m_eventHandler;
};

#include "Handle.inl"

} // Events
} // FW


#endif /* EVENTSVISUALIZERHANDLE_H_ */
