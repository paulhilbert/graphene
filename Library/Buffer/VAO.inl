/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


inline void VAO::bind() const {
	glBindVertexArray(m_id);
}

inline void VAO::release() const {
	glBindVertexArray(0);
}

inline void VAO::setAttrib(GLuint pos, GLuint dim, GLuint type) const {
	glVertexAttribPointer(pos, dim, type, GL_FALSE, 0, 0);
}

inline void VAO::enableAttrib(int pos) const {
	if (pos < 0) {
		std::cout << "Invalid attribute index in enableAttrib()." << std::endl;
		return;
	}
	glEnableVertexAttribArray((GLuint)pos);
}

inline void VAO::disableAttrib(int pos) const {
	if (pos < 0) {
		std::cout << "Invalid attribute index in disableAttrib()." << std::endl;
		return;
	}
	glDisableVertexAttribArray((GLuint)pos);
}
