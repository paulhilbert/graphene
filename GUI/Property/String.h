#ifndef PROPERTYSTRING_H_
#define PROPERTYSTRING_H_

#include "Base.h"
#include "Notify.h"
#include "Value.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

class String : public Base, public Notify<std::string>, public Value<std::string>, public Labeled {
	public:
		typedef std::shared_ptr<String> Ptr;
		typedef std::weak_ptr<String>   WPtr;
		using Notify<std::string>::Callback;

	public:
		String(std::string label);
		virtual ~String();
};

} // Property
} // GUI

#endif /* PROPERTYSTRING_H_ */
