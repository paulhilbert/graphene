#ifndef PROPERTYLABELED_H_
#define PROPERTYLABELED_H_

#include <string>

namespace GUI {
namespace Property {

class Labeled {
	public:
		Labeled(std::string label);
		virtual ~Labeled();

		virtual void setLabel(std::string label) = 0;

	protected:
		std::string m_label;
};

} // Property
} // GUI

#endif /* PROPERTYLABELED_H_ */
