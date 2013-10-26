/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef PROPERTYNUMBER_H_
#define PROPERTYNUMBER_H_

/**
 *  @file Number.h
 *
 *  Defines number property type.
 */

#include "Base.h"
#include "Notify.h"
#include "Value.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

/**
 *  Property type that provides means to set a number (integer or real) by input.
 *
 *  Backends usually implement this as a spinbox field (a field with a number input
 *  field and buttons to increment/decrement).
 *
 *  For integer number use setDigits(0). Using a higher number than 0 results in
 *  a real number with the corresponding number of digits after the period.
 *
 *  @see Range for another bounded input method.
 */
class Number : public Base, public Notify<void (double)>, public Value<double>, public Labeled {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Number> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Number>   WPtr;

		/** Specific callback type for this property */
		using Notify<void (double)>::Callback;

	public:
		/**
		 *  @internal Number(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		Number(std::string label);

		/**
		 *  @internal ~Number()
		 *
		 *  @brief Destructor
		 */
		virtual ~Number();

		/**
		 *  Set minimum value choosable.
		 *
		 *  @param min Minimum value to set.
		 */
		virtual void setMin(double min) = 0;

		/**
		 *  Set maximum value choosable.
		 *
		 *  @param max Maximum value to set.
		 */
		virtual void setMax(double max) = 0;

		/**
		 *  Set number of digits after the period (and therefore precision).
		 *
		 *  A parameter value of 0 results in a means to define integer values.
		 *
		 *  @param digits Number of digits after the period.
		 */
		virtual void setDigits(int digits) = 0;

		/**
		 *  Sets increment steps.
		 *
		 *  In case the backend implementation provides means (i.e. buttons) to
		 *  increment/decrement the number value, this method sets the amount
		 *  by which a single increment/decrement changes the number value.
		 *
		 *  @param step Increment/Decrement amount.
		 */
		virtual void setStep(double step) = 0;
};

} // Property
} // GUI

#endif /* PROPERTYNUMBER_H_ */
