/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef VAO_H_
#define VAO_H_

#include <iostream>

#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES
#endif
#include <GL/glew.h>
#include <vector>

#include "DataBuffer.h"

namespace Buffer {

class VAO {
	public:
		VAO();
		virtual ~VAO();

		void init();

		void bind() const;
		void release() const;

		void setAttrib(GLuint pos, GLuint dim, GLuint type = GL_FLOAT) const;
		void enableAttrib(int pos) const;
		void disableAttrib(int pos) const;

	protected:
		GLuint  m_id;
};

#include "VAO.inl"

} // Buffer

#endif /* DATABUFFER_H_ */
