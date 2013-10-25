#ifndef PROPERTYTOGGLEBUTTON_H_
#define PROPERTYTOGGLEBUTTON_H_

/**
 *  @file ToggleButton.h
 *
 *  Defines toggle button property type.
 */

#include "Base.h"
#include "Notify.h"
#include "Value.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

/**
 *  Property type that provides means to add a button with a static pressed state.
 *
 *  Backends usually implement this as a button that once pressed changes its state
 *  and keeps this state (i.e. stays down if enabled).
 *
 *  @see Bool For another way to input a boolean state.
 */
class ToggleButton : public Base, public Notify<void (bool)>, public Value<bool>, public Labeled {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<ToggleButton> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<ToggleButton>   WPtr;

		/** Specific callback type for this property */
		using Notify<void (bool)>::Callback;

	public:
		/**
		 *  @internal ToggleButton(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		ToggleButton(std::string label);

		/**
		 *  @internal ~ToggleButton()
		 *
		 *  @brief Destructor
		 */
		virtual ~ToggleButton();
};

} // Property
} // GUI

#endif /* PROPERTYTOGGLEBUTTON_H_ */
