#ifndef PROPERTYSEPARATOR_H_
#define PROPERTYSEPARATOR_H_

#include "Base.h"

namespace GUI {
namespace Property {

class Separator : public Base {
	public:
		typedef std::shared_ptr<Separator> Ptr;
		typedef std::weak_ptr<Separator>   WPtr;

	public:
		Separator();
		virtual ~Separator();
};

} // Property
} // GUI

#endif /* PROPERTYSEPARATOR_H_ */
