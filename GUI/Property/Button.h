#ifndef PROPERTYBUTTON_H_
#define PROPERTYBUTTON_H_

/**
 *  @file Button.h
 *
 *  Defines a property representing a clickable button.
 */

#include "Base.h"
#include "Notify.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

/**
 *  Button property.
 *
 *  This button is meant as a clickable no-state button.
 *  For a button having a static pressed-state see ToggleButton.
 */
class Button : public Base, public Notify<void (void)>, public Labeled {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Button> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Button>   WPtr;

		/** Specific callback type for this property */
		using Notify<void (void)>::Callback;

	public:
		/**
		 *  @internal Button(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		Button(std::string label);

		/**
		 *  @internal ~Button()
		 *
		 *  @brief Destructor
		 */
		virtual ~Button();
};

} // Property
} // GUI

#endif /* PROPERTYBUTTON_H_ */
