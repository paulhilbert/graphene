#ifndef PROPERTYNOTIFY_H_
#define PROPERTYNOTIFY_H_

#include <memory>
#include <functional>

namespace GUI {
namespace Property {

template <class... Params>
class Notify {
	public:
		typedef std::shared_ptr<Notify>      Ptr;
		typedef std::weak_ptr<Notify>        WPtr;
		typedef std::function<void (Params...)>  Callback;

	public:
		Notify();
		virtual ~Notify();

		void setCallback(Callback onChange);
		void unsetCallback();

	protected:
		void notify(Params... params) const;

	protected:
		Callback m_onChange;
};

#include "Notify.inl"

} // Property
} // GUI

#endif /* PROPERTYNOTIFY_H_ */
