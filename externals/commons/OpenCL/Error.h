#ifndef OPENCLERROR_H_
#define OPENCLERROR_H_

#include <boost/lexical_cast.hpp>
#include "../IO/Log.h"
#include "../Testing/BaseException.h"

namespace OpenCL {
namespace Error {

class OutOfMemAlloc : public ::Testing::BaseException {
	public:
		OutOfMemAlloc(float mbRequested, float mbAvailable) : ::Testing::BaseException() {
			m_msg = "Not enough memory on device. Requested: "+lexical_cast<std::string>(mbRequested)+"MiB. Available: "+lexical_cast<std::string>(mbAvailable)+"MiB.";
		}

		virtual ~OutOfMemAlloc() {
		}

		void print() const noexcept override {
			IO::Log::error(m_msg);
		}

	protected:
		std::string m_msg;
};


} // Error
} // OpenCL


#endif /* OPENCLERROR_H_ */
