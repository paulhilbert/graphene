#include "Event.h"

namespace FW {

Event::Event(std::string id) : Visualizer(id) {
}

Event::~Event() {
}

void Event::init() {
	addProperties();
	registerEvents();
}

void Event::render() {
}

void Event::addProperties() {
}

void Event::registerEvents() {
	fw()->events()->connect<void (int, int)>("LEFT_CLICK", std::bind(&Event::click, this, std::placeholders::_1, std::placeholders::_2));
	fw()->events()->connect<void (int, int)>("RIGHT_CLICK", std::bind(&Event::click, this, std::placeholders::_1, std::placeholders::_2));
	fw()->events()->connect<void (int, int, int, int)>("MOVE", std::bind(&Event::move, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
	fw()->events()->connect<void (int, int)>("WINDOW_RESIZE", std::bind(&Event::resize, this, std::placeholders::_1, std::placeholders::_2));
	fw()->events()->connect<void (int)>("SCROLL", std::bind(&Event::scroll, this, std::placeholders::_1));
	fw()->events()->connect<void (Keys::Modifier)>("MODIFIER_PRESS", std::bind(&Event::modifier, this, std::placeholders::_1, "pressed"));
	fw()->events()->connect<void (Keys::Modifier)>("MODIFIER_RELEASE", std::bind(&Event::modifier, this, std::placeholders::_1, "released"));
	fw()->events()->connect<void (std::string)>("CHAR_KEY_PRESS", std::bind(&Event::charKey, this, std::placeholders::_1, "pressed"));
	fw()->events()->connect<void (std::string)>("CHAR_KEY_RELEASE", std::bind(&Event::charKey, this, std::placeholders::_1, "released"));
	fw()->events()->connect<void (Keys::Special)>("SPECIAL_KEY_PRESS", std::bind(&Event::specialKey, this, std::placeholders::_1, "pressed"));
	fw()->events()->connect<void (Keys::Special)>("SPECIAL_KEY_RELEASE", std::bind(&Event::specialKey, this, std::placeholders::_1, "released"));
}

void Event::move(int dx, int dy, int x, int y) {
	gui()->status()->set("move coords: " + lexical_cast<std::string>(x) + ", " + lexical_cast<std::string>(y));
}

void Event::click(int x, int y) {
	gui()->status()->set("click coords: " + lexical_cast<std::string>(x) + ", " + lexical_cast<std::string>(y));
}

void Event::resize(int w, int h) {
	gui()->status()->set("new size: " + lexical_cast<std::string>(w) + ", " + lexical_cast<std::string>(h));
}

void Event::scroll(int delta) {
	gui()->status()->set("scroll delta: " + lexical_cast<std::string>(delta));
}

void Event::modifier(Keys::Modifier mod, std::string action) {
	switch (mod) {
		case Keys::SHIFT: gui()->status()->set("modifier "+action+": Shift"); break;
		case Keys::CTRL: gui()->status()->set("modifier "+action+": Control"); break;
		case Keys::ALT: gui()->status()->set("modifier "+action+": Alt"); break;
		default: gui()->status()->set("modifier "+action+": AltGr"); break;
	}
}

void Event::charKey(std::string key, std::string action) {
	gui()->status()->set("char key "+action+": \""+key+"\"");
}

void Event::specialKey(Keys::Special key, std::string action) {
	switch (key) {
		case Keys::ESCAPE: gui()->status()->set("special key "+action+": ESCAPE"); break;
		case Keys::TAB: gui()->status()->set("special key "+action+": TAB"); break;
		case Keys::BACKTAB: gui()->status()->set("special key "+action+": BACKTAB"); break;
		case Keys::BACKSPACE: gui()->status()->set("special key "+action+": BACKSPACE"); break;
		case Keys::RETURN: gui()->status()->set("special key "+action+": RETURN"); break;
		case Keys::ENTER: gui()->status()->set("special key "+action+": ENTER"); break;
		case Keys::INSERT: gui()->status()->set("special key "+action+": INSERT"); break;
		case Keys::DELETE: gui()->status()->set("special key "+action+": DELETE"); break;
		case Keys::PAUSE: gui()->status()->set("special key "+action+": PAUSE"); break;
		case Keys::PRINT: gui()->status()->set("special key "+action+": PRINT"); break;
		case Keys::SYSREQ: gui()->status()->set("special key "+action+": SYSREQ"); break;
		case Keys::CLEAR: gui()->status()->set("special key "+action+": CLEAR"); break;
		case Keys::HOME: gui()->status()->set("special key "+action+": HOME"); break;
		case Keys::END: gui()->status()->set("special key "+action+": END"); break;
		case Keys::LEFT: gui()->status()->set("special key "+action+": LEFT"); break;
		case Keys::UP: gui()->status()->set("special key "+action+": UP"); break;
		case Keys::RIGHT: gui()->status()->set("special key "+action+": RIGHT"); break;
		case Keys::DOWN: gui()->status()->set("special key "+action+": DOWN"); break;
		case Keys::PAGEUP: gui()->status()->set("special key "+action+": PAGEUP"); break;
		case Keys::PAGEDOWN: gui()->status()->set("special key "+action+": PAGEDOWN"); break;
		case Keys::META: gui()->status()->set("special key "+action+": META"); break;
		case Keys::CAPSLOCK: gui()->status()->set("special key "+action+": CAPSLOCK"); break;
		case Keys::NUMLOCK: gui()->status()->set("special key "+action+": NUMLOCK"); break;
		case Keys::SCROLLLOCK: gui()->status()->set("special key "+action+": SCROLLLOCK"); break;
		case Keys::F1: gui()->status()->set("special key "+action+": F1"); break;
		case Keys::F2: gui()->status()->set("special key "+action+": F2"); break;
		case Keys::F3: gui()->status()->set("special key "+action+": F3"); break;
		case Keys::F4: gui()->status()->set("special key "+action+": F4"); break;
		case Keys::F5: gui()->status()->set("special key "+action+": F5"); break;
		case Keys::F6: gui()->status()->set("special key "+action+": F6"); break;
		case Keys::F7: gui()->status()->set("special key "+action+": F7"); break;
		case Keys::F8: gui()->status()->set("special key "+action+": F8"); break;
		case Keys::F9: gui()->status()->set("special key "+action+": F9"); break;
		case Keys::F10: gui()->status()->set("special key "+action+": F10"); break;
		case Keys::F11: gui()->status()->set("special key "+action+": F11"); break;
		case Keys::F12: gui()->status()->set("special key "+action+": F12"); break;
		default: break;
	}
}

Event::Factory::Factory() : FW::Factory() {
}

Event::Factory::~Factory() {
}

void Event::Factory::init() {
}

Visualizer::Ptr Event::Factory::addVisualizer() {
	std::string name = gui()->properties()->get<String>({"__name__"})->value();
	Event::Ptr vis(new Event(name));
	return std::dynamic_pointer_cast<Visualizer>(vis);
}

} // FW

