#ifndef KEYS_H_
#define KEYS_H_

namespace FW {
namespace Events {

struct Keys {
	typedef enum {
		CTRL,
		ALT,
		SHIFT,
		ALTGR
	} Modifier;

	typedef enum {
		ESCAPE,
		TAB,
		BACKTAB,
		BACKSPACE,
		RETURN,
		ENTER,
		INSERT,
		DEL,
		PAUSE,
		PRINT,
		SYSREQ,
		CLEAR,
		HOME,
		END,
		LEFT,
		UP,
		RIGHT,
		DOWN,
		PAGEUP,
		PAGEDOWN,
		META,
		CAPSLOCK,
		NUMLOCK,
		SCROLLLOCK,
		F1,
		F2,
		F3,
		F4,
		F5,
		F6,
		F7,
		F8,
		F9,
		F10,
		F11,
		F12
	} Special;
};

} // Events
} // FW

#endif /* KEYS_H_ */
