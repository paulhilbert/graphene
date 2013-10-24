#ifndef FWMODIFIER_H_
#define FWMODIFIER_H_

/**
 *  @file Modifier.h
 *
 *  @brief Specifies handle to access modifier states.
 *
 */

#include <include/common.h>

namespace FW {
namespace Events {

/**
 *  @brief Access class for modifier states.
 *
 *  This class is supplied to visualizers in order to
 *  check modifier states at runtime
 *
 *  Example code:
 *
 *      void CustomVisualizer::registerEvents() {
 *          fw()->events()->connect<void (int,int,int,int)("LEFT_DRAG", [&] (int dx, int dy, int x, int y) {
 *              if (fw()->modifier()->shift()) {
 *                  gui()->status()->set("Dragging while pressing shift");
 *              } else {
 *                  gui()->status()->set("Dragging while not pressing shift");
 *              }
 *          });
 *      }
 *
 *  This code would print the shift state to the status bar while dragging with the left mouse button.
 *
 */
class Modifier {
	public:
		typedef std::shared_ptr<Modifier> Ptr;
		typedef std::weak_ptr<Modifier>   WPtr;

	public:
		/**
		 *  Constructor
		 */
		Modifier();
		/**
		 *  Destructor
		 */
		virtual ~Modifier();

		/**
		 *  Return ctrl modifier state.
		 *  @return Returns true if ctrl button is currently pressed.
		 */
		bool& ctrl();

		/**
		 *  Return shift modifier state.
		 *  @return Returns true if shift button is currently pressed.
		 */
		bool& shift();

		/**
		 *  Return alt modifier state.
		 *  @return Returns true if alt button is currently pressed.
		 */
		bool& alt();

		/**
		 *  Return altGr modifier state.
		 *  @return Returns true if altGr button is currently pressed.
		 */
		bool& altgr();

	protected:
		bool m_ctrl;
		bool m_shift;
		bool m_alt;
		bool m_altgr;
};

} // Events
} // FW

#endif /* FWMODIFIER_H_ */
