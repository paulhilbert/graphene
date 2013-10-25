#ifndef PROPERTYGROUP_H_
#define PROPERTYGROUP_H_

/**
 *  @file PropGroup.h
 *
 *  Defines property group type.
 */

#include "Container.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

/**
 *  Property type that provides means to group subproperties.
 *
 *  Backends usually implement this as a framed, labeled box
 *  that groups all properties within.
 *
 *  @see Section for a collapsable version of Group.
 */
class Group : public Container, public Labeled {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Group> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Group>   WPtr;

	public:
		/**
		 *  @internal Group(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		Group(std::string label);

		/**
		 *  @internal ~Group()
		 *
		 *  @brief Destructor
		 */
		virtual ~Group();
};

} // Property
} // GUI

#endif /* PROPERTYGROUP_H_ */
