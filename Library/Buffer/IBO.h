/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


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
