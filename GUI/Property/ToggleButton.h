#ifndef PROPERTYTOGGLEBUTTON_H_
#define PROPERTYTOGGLEBUTTON_H_

#include "Base.h"
#include "Notify.h"
#include "Value.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

class ToggleButton : public Base, public Notify<void (bool)>, public Value<bool>, public Labeled {
	public:
		typedef std::shared_ptr<ToggleButton> Ptr;
		typedef std::weak_ptr<ToggleButton>   WPtr;
		using typename Notify<void (bool)>::Callback;

	public:
		ToggleButton(std::string label);
		virtual ~ToggleButton();
};

} // Property
} // GUI

#endif /* PROPERTYTOGGLEBUTTON_H_ */
