#ifndef PROPERTYBOOL_H_
#define PROPERTYBOOL_H_

/**
 *  @file Bool.h
 *
 *  Defines a property to choose a boolean state.
 */

#include "Base.h"
#include "Value.h"
#include "Notify.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

/**
 *  Boolean state property.
 *
 *  Backends usually implement this as a checkbox.
 *
 *  @see ToggleButton for another way to input a boolean state.
 */
class Bool : public Base, public Value<bool>, public Notify<void (bool)>, public Labeled {
	public:
		/** Shared pointer to this */
		typedef std::shared_ptr<Bool> Ptr;

		/** Weak pointer to this */
		typedef std::weak_ptr<Bool>   WPtr;

		/** Specific callback type for this property */
		using Notify<void (bool)>::Callback;

	public:
		/**
		 *  @internal Bool(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		Bool(std::string label);

		/**
		 *  @internal ~Bool()
		 *
		 *  @brief Destructor.
		 */
		virtual ~Bool();
};

} // Property
} // GUI

#endif /* PROPERTYBOOL_H_ */
