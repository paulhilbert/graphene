#ifndef PROPERTYGROUP_H_
#define PROPERTYGROUP_H_

#include "Container.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

class Group : public Container, public Labeled {
	public:
		typedef std::shared_ptr<Group> Ptr;
		typedef std::weak_ptr<Group>   WPtr;

	public:
		Group(std::string label);
		virtual ~Group();
};

} // Property
} // GUI

#endif /* PROPERTYGROUP_H_ */
