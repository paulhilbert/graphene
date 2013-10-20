#ifndef PROPERTYVALUE_H_
#define PROPERTYVALUE_H_

#include <include/common.h>

namespace GUI {
namespace Property {

template <class Type>
class Value {
	public:
		typedef std::shared_ptr<Value<Type>>  Ptr;
		typedef std::weak_ptr<Value<Type>>    WPtr;

	public:
		Value();
		virtual ~Value();

		virtual Type value() const = 0;
		virtual void setValue(Type value) = 0;
};

#include "Value.inl"

} // Property
} // GUI

#endif /* PROPERTYVALUE_H_ */
