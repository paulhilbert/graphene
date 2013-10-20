#ifndef GUIELEMENT_H_
#define GUIELEMENT_H_

#include <include/common.h>

namespace GUI {

class GUIElement {
	public:
		typedef std::shared_ptr<GUIElement> Ptr;
		typedef std::weak_ptr<GUIElement> WPtr;

	public:
		GUIElement();
		virtual ~GUIElement();

		virtual void show()          = 0;
		virtual void hide()          = 0;
		virtual bool visible() const = 0;

		virtual void enable()        = 0;
		virtual void disable()       = 0;
		virtual bool enabled() const = 0;
};

} // GUI

#endif /* GUIELEMENT_H_ */
