#ifndef PROPERTYRANGE_H_
#define PROPERTYRANGE_H_

#include "Base.h"
#include "Notify.h"
#include "Value.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

class Range : public Base, public Notify<void (double)>, public Value<double>, public Labeled {
	public:
		typedef std::shared_ptr<Range> Ptr;
		typedef std::weak_ptr<Range>   WPtr;
		using Notify<void (double)>::Callback;

	public:
		Range(std::string label);
		virtual ~Range();

		virtual void setMin(double min) = 0;
		virtual void setMax(double max) = 0;
		virtual void setDigits(int digits) = 0;
};

} // Property
} // GUI

#endif /* PROPERTYRANGE_H_ */
