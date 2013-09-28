#ifndef PROPERTYBASE_H_
#define PROPERTYBASE_H_

#include <iostream> // for graphics
#include <memory>

#include "../GUIElement.h"

namespace GUI {
namespace Property {

class Container;

class Base : public GUIElement {
	public:
		typedef std::shared_ptr<Base> Ptr;
		typedef std::weak_ptr<Base>   WPtr;

	public:
		Base();
		virtual ~Base();

		virtual bool isContainer() const;
};

} // Property
} // GUI

#endif /* PROPERTYBASE_H_ */
