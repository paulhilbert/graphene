#ifndef VAO_H_
#define VAO_H_

#include <iostream>

#include <IO/Log.h>
using namespace IO;

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
