#ifndef PROPERTYRANGE_H_
#define PROPERTYRANGE_H_

/**
 *  @file Range.h
 *
 *  Defines number range property type.
 */

#include "Base.h"
#include "Notify.h"
#include "Value.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

/**
 *  Property type that provides means to choose a number in a given range.
 *
 *  Backends usually implement this as a slider widget.
 *
 *  For integer number use setDigits(0). Using a higher number than 0 results in
 *  a real number with the corresponding number of digits after the period.
 *
 *  @see Number for another way to have a bounded number input.
 */
class Range : public Base, public Notify<void (double)>, public Value<double>, public Labeled {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Range> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Range>   WPtr;

		/** Specific callback type for this property */
		using Notify<void (double)>::Callback;

	public:
		/**
		 *  @internal Range(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		Range(std::string label);

		/**
		 *  @internal ~Range()
		 *
		 *  @brief Destructor
		 */
		virtual ~Range();

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
};

} // Property
} // GUI

#endif /* PROPERTYRANGE_H_ */
