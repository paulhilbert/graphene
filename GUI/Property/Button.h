#ifndef PROPERTYBUTTON_H_
#define PROPERTYBUTTON_H_

#include "Base.h"
#include "Notify.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

class Button : public Base, public Notify<>, public Labeled {
	public:
		typedef std::shared_ptr<Button> Ptr;
		typedef std::weak_ptr<Button>   WPtr;
		using typename Notify<>::Callback;

	public:
		Button(std::string label);
		virtual ~Button();
};

} // Property
} // GUI

#endif /* PROPERTYBUTTON_H_ */
