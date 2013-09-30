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
		Log::warn("Invalid attribute index in enableAttrib().");
		return;
	}
	glEnableVertexAttribArray((GLuint)pos);
}

inline void VAO::disableAttrib(int pos) const {
	if (pos < 0) {
		Log::warn("Invalid attribute index in disableAttrib().");
		return;
	}
	glDisableVertexAttribArray((GLuint)pos);
}
