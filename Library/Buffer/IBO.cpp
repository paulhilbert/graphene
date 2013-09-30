#include "IBO.h"

namespace Buffer {


IBO::IBO() : DataBuffer<GLuint>() {
}

IBO::~IBO() {
}

void IBO::upload(AccessMethod aMethod) {
	this->uploadData(GL_ELEMENT_ARRAY_BUFFER, aMethod);
}

void IBO::bind() {
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->m_id);
}

void IBO::release() {
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}


} // Buffer
