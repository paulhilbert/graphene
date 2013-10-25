#ifndef PROPERTYSEPARATOR_H_
#define PROPERTYSEPARATOR_H_

/**
 *  @file Separator.h
 *
 *  Defines separator property type.
 */

#include "Base.h"

namespace GUI {
namespace Property {

/**
 *  Property type that simply renders a horizontal separator.
 *
 *  Backends usually implement this as a simple horizontal line.
 *
 */
class Separator : public Base {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Separator> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Separator>   WPtr;

	public:
		/**
		 *  @internal Separator()
		 *
		 *  @brief Constructor
		 */
		Separator();

		/**
		 *  @internal ~Separator()
		 *
		 *  @brief Destructor
		 */
		virtual ~Separator();
};

} // Property
} // GUI

#endif /* PROPERTYSEPARATOR_H_ */
