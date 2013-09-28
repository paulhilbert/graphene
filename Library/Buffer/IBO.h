#ifndef IBO_H_
#define IBO_H_

#include "DataBuffer.h"

namespace Buffer {

class IBO : public DataBuffer<GLuint> {
	public:
		IBO();
		virtual ~IBO();

		void upload(AccessMethod aMethod);
		void bind();
		void release();
};


} // Buffer

#endif /* IBO_H_ */
