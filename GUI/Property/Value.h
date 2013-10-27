/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef PROPERTYVALUE_H_
#define PROPERTYVALUE_H_

/**
 *  @file Value.h
 *
 *  Defines base valued property type.
 */

#include <include/common.h>

namespace GUI {
namespace Property {

/**
 *  Property base type that provides means to set and get inherent property values.
 *
 *  Properties derived from Value<Type> have an inherent value Type modified by
 *  the property. For example the String property is derived from Value<Type> in order
 *  to have value() and setValue() methods to get and set the string value inside.
 *
 *  @tparam Type Value type of the derived property.
 */
template <class Type>
class Value {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Value<Type>>  Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Value<Type>>    WPtr;

	public:
		/**
		 *  @internal Value()
		 *
		 *  @brief Constructor
		 */
		Value();

		/**
		 *  @internal ~Value()
		 *
		 *  @brief Destructor
		 */
		virtual ~Value();

		/**
		 *  Get inherent value.
		 *
		 *  @return Value (that is: state) of this property.
		 */
		virtual Type value() const = 0;

		/**
		 *  Set inherent value.
		 *
		 *  @param value New value of this property.
		 */
		virtual void setValue(Type value) = 0;
};

#include "Value.inl"

} // Property
} // GUI

#endif /* PROPERTYVALUE_H_ */
