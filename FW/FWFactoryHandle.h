#ifndef FWFACTORYHANDLE_H_
#define FWFACTORYHANDLE_H_

#include <memory>

namespace FW {

class FactoryHandle {
	public:
		typedef std::shared_ptr<FactoryHandle> Ptr;
		struct Impl;

	public:
		FactoryHandle();
		virtual ~FactoryHandle();

	private:
		std::shared_ptr<Impl> m_impl;
};

} // FW

#endif /* FWFACTORYHANDLE_H_ */
