#ifndef PROPERTYNUMBER_H_
#define PROPERTYNUMBER_H_

#include "Base.h"
#include "Notify.h"
#include "Value.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

class Number : public Base, public Notify<void (double)>, public Value<double>, public Labeled {
	public:
		typedef std::shared_ptr<Number> Ptr;
		typedef std::weak_ptr<Number>   WPtr;
		using Notify<void (double)>::Callback;

	public:
		Number(std::string label);
		virtual ~Number();

		virtual void setMin(double min) = 0;
		virtual void setMax(double max) = 0;
		virtual void setDigits(int digits) = 0;
		virtual void setStep(double step) = 0;
};

} // Property
} // GUI

#endif /* PROPERTYNUMBER_H_ */
