inline Texture::Texture() : m_id(0) {
	glGenTextures(1, &m_id);
}

inline Texture::Texture(GLenum iformat, int width, int height, GLfloat *pixels) : m_id(0), m_boundToRB(false) {
	glGenTextures(1, &m_id);
	bind();
	load(iformat, width, height, pixels);
}

inline Texture::Texture(GLenum iformat, int width, int height, GLubyte *pixels) : m_id(0) {
	glGenTextures(1, &m_id);
	bind();
	load(iformat, width, height, pixels);
}

inline Texture::~Texture() {
	if (m_boundToRB) glDeleteRenderbuffers(1, &m_id);
	glDeleteTextures(1, &m_id);
}

inline void Texture::setFiltering(GLenum filter) {
	bind();
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
}

inline void Texture::bind() {
	glBindTexture(GL_TEXTURE_2D, m_id);
}

inline void Texture::unbind() {
	glBindTexture(GL_TEXTURE_2D, 0);
}

inline void Texture::bindToRenderbuffer(GLuint attachment) {
	glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, attachment, GL_TEXTURE_2D, m_id, 0);
	m_boundToRB = true;
}

inline GLuint Texture::id() const {
	return m_id;
}

inline void Texture::load(GLenum iformat, int width, int height, GLfloat *pixels) {
	GLenum format;

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	if (iformat == GL_RGB16F || iformat == GL_RGB32F) {
		format = GL_RGB;
	}
	else if (iformat == GL_RGBA16F || iformat == GL_RGBA32F) {
		format = GL_RGBA;
	}
	else if (iformat == GL_RGBA8 || iformat == GL_RGBA || iformat == 4) {
		format = GL_RGBA;
	}
	else if (iformat == GL_RGB8 || iformat == GL_RGB || iformat == 3) {
		format = GL_RGB;
	}
	else if (iformat == GL_LUMINANCE8_ALPHA8 || iformat == GL_LUMINANCE_ALPHA || iformat == 2) {
		format = GL_LUMINANCE_ALPHA;
	}
	else if (iformat == GL_LUMINANCE8 || iformat == GL_LUMINANCE || iformat == 1) {
		format = GL_LUMINANCE;
	}
	else if (iformat == GL_DEPTH_COMPONENT || iformat == GL_DEPTH_COMPONENT16 || 
		iformat == GL_DEPTH_COMPONENT24 || iformat == GL_DEPTH_COMPONENT32 || iformat == GL_DEPTH_COMPONENT32F) {
		format = GL_DEPTH_COMPONENT;
	}
	else {
		throw std::invalid_argument("Texture::load - Unknown internal format");
	}

	glTexImage2D(GL_TEXTURE_2D, 0, iformat, width, height, 0, format, GL_FLOAT, pixels);
}

inline void Texture::load(GLenum iformat, int width, int height, GLubyte *pixels) {
	GLenum format;

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	if (iformat == GL_RGB8) {
		format = GL_RGB;
	}
	else if (iformat == GL_RGBA8) {
		format = GL_RGBA;
	}
	else {
		throw std::invalid_argument("Texture::load - Unknown internal format");
	}

	glTexImage2D(GL_TEXTURE_2D, 0, iformat, width, height, 0, format, GL_UNSIGNED_BYTE, pixels);
}

