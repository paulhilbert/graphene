#ifndef PROPERTYSECTION_H_
#define PROPERTYSECTION_H_

#include "Container.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

class Section : public Container, public Labeled {
	public:
		typedef std::shared_ptr<Section> Ptr;
		typedef std::weak_ptr<Section>   WPtr;

	public:
		Section(std::string label);
		virtual ~Section();

		void collapse();
		void expand();
		virtual void setCollapsed(bool collapsed) = 0;
};

} // Property
} // GUI

#endif /* PROPERTYSECTION_H_ */
