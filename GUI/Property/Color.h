#ifndef PROPERTYCOLOR_H_
#define PROPERTYCOLOR_H_

#include <include/common.h>

#include "Base.h"
#include "Notify.h"
#include "Value.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

class Color : public Base, public Notify<void (Eigen::Vector4f)>, public Value<Eigen::Vector4f>, public Labeled {
	public:
		typedef std::shared_ptr<Color> Ptr;
		typedef std::weak_ptr<Color>   WPtr;
		using Notify<void (Eigen::Vector4f)>::Callback;

	public:
		Color(std::string label);
		virtual ~Color();
};


} // Property
} // GUI


#endif /* PROPERTYCOLOR_H_ */
