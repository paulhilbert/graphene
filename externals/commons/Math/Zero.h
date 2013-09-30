#ifndef ZERO_H_
#define ZERO_H_

#include <limits>
#include <cmath>

/*
 * Welcome to the most trivial class that is actually useful - an
 * implementation of "basically zero in a way"
 *
 * Warning: Usage may increase lazyness immensively.
 *
 * Synopsis:
 *
 * 	#include <Generic/Zero.h>
 * 	using Generic::Zero;
 *
 * 	...
 *
 * 	if (variable == Zero()) { // boils down to "abs(variable) < epsilon"
 *			// at this point either variable or -variable
 *			// are <= eps
 *			// so one may safely assume it to be Zero
 * 	}
 *
 * 	...
 *
 * 	if (otherVariable != Zero()) {
 *			// You may safely divide by otherVariable!
 * 	}
 *
 *  Works for any variable of type T, where numeric_limits<T>::epsilon()
 *  is defined. However only floating point variables make sense...
 *
*/

namespace Math {

class Zero {
	public:
		Zero() {}
		virtual ~Zero() {}

		template <class Scalar>
		bool operator ==(Scalar op);

		template <class Scalar>
		bool operator !=(Scalar op);
};

template <class Scalar>
bool operator==(Scalar op, Zero z);

template <class Scalar>
bool operator!=(Scalar op, Zero z);

#include "Zero.inl"

} // Math

#endif /* ZERO_H_ */
