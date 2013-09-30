#ifndef PROPERTYBOOL_H_
#define PROPERTYBOOL_H_

#include "Base.h"
#include "Value.h"
#include "Notify.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

class Bool : public Base, public Value<bool>, public Notify<void (bool)>, public Labeled {
	public:
		typedef std::shared_ptr<Bool> Ptr;
		typedef std::weak_ptr<Bool>   WPtr;
		using Notify<void (bool)>::Callback;

	public:
		Bool(std::string label);
		virtual ~Bool();
};

} // Property
} // GUI

#endif /* PROPERTYBOOL_H_ */
