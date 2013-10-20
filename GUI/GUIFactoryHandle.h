#ifndef GUIFACTORYHANDLE_H_
#define GUIFACTORYHANDLE_H_

#include <include/common.h>

#include "Property/Container.h"

namespace GUI {

class FactoryHandle {
	public:
		typedef std::shared_ptr<FactoryHandle> Ptr;

	public:
		FactoryHandle(Property::Container::Ptr properties);
		virtual ~FactoryHandle();

		Property::Container::Ptr properties();

	protected:
		Property::Container::Ptr m_properties;
};


} // GUI

#endif /* GUIFACTORYHANDLE_H_ */
