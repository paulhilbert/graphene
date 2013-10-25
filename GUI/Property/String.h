#ifndef PROPERTYSTRING_H_
#define PROPERTYSTRING_H_

/**
 *  @file String.h
 *
 *  Defines string property type.
 */

#include "Base.h"
#include "Notify.h"
#include "Value.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

/**
 *  Property type that provides means to enter/modify a string.
 *
 *  Backends usually implement this as a one-line editable text field.
 */
class String : public Base, public Notify<void (std::string)>, public Value<std::string>, public Labeled {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<String> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<String>   WPtr;

		/** Specific callback type for this property */
		using Notify<void (std::string)>::Callback;

	public:
		/**
		 *  @internal String(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		String(std::string label);

		/**
		 *  @internal ~String()
		 *
		 *  @brief Destructor
		 */
		virtual ~String();
};

} // Property
} // GUI

#endif /* PROPERTYSTRING_H_ */
