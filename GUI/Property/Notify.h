#ifndef PROPERTYNOTIFY_H_
#define PROPERTYNOTIFY_H_

#include <include/common.h>

namespace GUI {
namespace Property {

template <class Sig>
class Notify {
	public:
		typedef std::shared_ptr<Notify>      Ptr;
		typedef std::weak_ptr<Notify>        WPtr;
		typedef std::function<Sig>  Callback;

	public:
		Notify();
		virtual ~Notify();

		void setCallback(Callback onChange);
		void unsetCallback();

		void notify() const;
		template <class Arg0>
		void notify(Arg0 arg0) const;
		template <class Arg0, class Arg1>
		void notify(Arg0 arg0, Arg1 arg1) const;

	protected:
		Callback m_onChange;
};

#include "Notify.inl"

} // Property
} // GUI

#endif /* PROPERTYNOTIFY_H_ */
