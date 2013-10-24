#ifndef KEYS_H_
#define KEYS_H_

/**
 *  @file Keys.h
 *
 *  @brief Specifies codes for keyboard keys.
 *
 */

namespace FW {
namespace Events {

/**
 *  @brief Type class for key types
 */
struct Keys {
	/**
	 *  @brief Modifier key codes
	 */
	typedef enum {
		CTRL, ///< ctrl key
		ALT,  ///< alt key
		SHIFT,  ///< shift key
		ALTGR
	} Modifier;

	/**
	 *  @brief Special key codes
	 */
	typedef enum {
		ESCAPE,  ///< escape key
		TAB,  ///< tab key
		BACKTAB,  ///< backtab key
		BACKSPACE,  ///< backspace key
		RETURN,  ///< return key
		ENTER,  ///< enter key
		INSERT,  ///< insert key
		DEL,  ///< del key
		PAUSE,  ///< pause key
		PRINT,  ///< print key
		SYSREQ,  ///< sysreq key
		CLEAR,  ///< clear key
		HOME,  ///< home key
		END,  ///< end key
		LEFT,  ///< left key
		UP,  ///< up key
		RIGHT,  ///< right key
		DOWN,  ///< down key
		PAGEUP,  ///< pageup key
		PAGEDOWN,  ///< pagedown key
		META,  ///< meta key
		CAPSLOCK,  ///< capslock key
		NUMLOCK,  ///< numlock key
		SCROLLLOCK,  ///< scrolllock key
		F1,  ///< f1 key
		F2,  ///< f2 key
		F3,  ///< f3 key
		F4,  ///< f4 key
		F5,  ///< f5 key
		F6,  ///< f6 key
		F7,  ///< f7 key
		F8,  ///< f8 key
		F9,  ///< f9 key
		F10,  ///< f10 key
		F11,  ///< f11 key
		F12   ///< f12 key
	} Special;
};

} // Events
} // FW

#endif /* KEYS_H_ */
