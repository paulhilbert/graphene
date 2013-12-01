/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


inline IBO::IBO() : DataBuffer<GLuint>() {
}

inline IBO::~IBO() {
}

inline void IBO::upload(AccessMethod aMethod) {
	this->uploadData(GL_ELEMENT_ARRAY_BUFFER, aMethod);
}

inline void IBO::bind() {
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->m_id);
}

inline void IBO::release() {
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}
