/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


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
