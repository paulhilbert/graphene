inline Texture::Texture() : m_id(0) {
	glGenTextures(1, &m_id);
}

inline Texture::Texture(GLenum iformat, int width, int height, GLfloat *pixels) : m_id(0) {
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

inline void Texture::load(GLenum iformat, int width, int height, GLfloat *pixels) {
	GLenum format;

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	if (iformat == GL_RGB16F_ARB || iformat == GL_RGB32F_ARB) {
		format = GL_RGB;
	}
	else if (iformat == GL_RGBA16F_ARB || iformat == GL_RGB32F_ARB) {
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
	else if (iformat == GL_DEPTH_COMPONENT || iformat == GL_DEPTH_COMPONENT16_ARB || 
		iformat == GL_DEPTH_COMPONENT24_ARB || iformat == GL_DEPTH_COMPONENT32_ARB) {
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

