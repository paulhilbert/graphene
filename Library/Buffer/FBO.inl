/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


inline FBO::FBO() : m_width(0), m_height(0), m_frame_id(0), m_depth_id(0), m_stencil_id(0), m_buffers(0) {
	glGetIntegerv(GL_MAX_COLOR_ATTACHMENTS, &m_max_color_attachments);
	m_buffers = new GLenum[m_max_color_attachments];

	glGenFramebuffers(1, &m_frame_id);
}

inline FBO::FBO(int width, int height) : m_width(0), m_height(0), m_frame_id(0), m_depth_id(0), m_stencil_id(0), m_buffers(0) {
	glGetIntegerv(GL_MAX_COLOR_ATTACHMENTS, &m_max_color_attachments);
	m_buffers = new GLenum[m_max_color_attachments];

	glGenFramebuffers(1, &m_frame_id);
	m_width = width;
	m_height = height;
}

inline FBO::~FBO() {
	GLuint tex_id;

	std::vector<GLuint>::const_iterator cii;
	for(cii = m_tex_id.begin(); cii != m_tex_id.end(); cii++) {
		tex_id = *cii;
		glDeleteTextures(1, &tex_id);
	}

	if (m_depth_id) {
		glDeleteRenderbuffers(1, &m_depth_id);
	}
	if (m_stencil_id) {
		glDeleteRenderbuffers(1, &m_stencil_id);
	}
	glDeleteFramebuffers(1, &m_frame_id);
	delete[] m_buffers;
}

inline void FBO::setSize(int width, int height) {
	m_width = width; m_height = height;
};

inline void FBO::attachRender(GLenum iformat) {
	GLenum attachment;
	GLuint render_id;

	if (m_width == 0 || m_height == 0) {
		throw std::domain_error("FBO::attachRender - one of the dimensions is zero");
	}

	if (iformat == GL_DEPTH_COMPONENT24 || iformat == GL_DEPTH_COMPONENT) {
		attachment = GL_DEPTH_ATTACHMENT;
	}
	else if (iformat == GL_STENCIL_INDEX1 || iformat == GL_STENCIL_INDEX4 || iformat == GL_STENCIL_INDEX8 ||
		iformat == GL_STENCIL_INDEX16 || iformat == GL_STENCIL_INDEX) {
		attachment = GL_STENCIL_ATTACHMENT;
	}
	else if (iformat == GL_DEPTH24_STENCIL8 || iformat == GL_DEPTH_STENCIL) {
		attachment = GL_DEPTH_STENCIL_ATTACHMENT;
	}
	else {
		throw std::invalid_argument("FBO::attachRender - unrecognized internal format");
	}

	glGenRenderbuffers(1, &render_id);
	glBindFramebuffer(GL_FRAMEBUFFER, m_frame_id);
	glBindRenderbuffer(GL_RENDERBUFFER, render_id);
	glRenderbufferStorage(GL_RENDERBUFFER, iformat, m_width, m_height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, attachment, GL_RENDERBUFFER, render_id);

	if (attachment == GL_DEPTH_ATTACHMENT || attachment == GL_DEPTH_STENCIL_ATTACHMENT) {
		m_depth_id = render_id;
	}
	else if (attachment == GL_STENCIL_ATTACHMENT) {
		m_stencil_id = render_id;
	}
}

inline void FBO::attachTexture(GLenum iformat, GLint filter) {
	GLenum format;
	GLenum type;
	GLenum attachment;
	GLuint tex_id;

	if (m_width == 0 || m_height == 0) {
		throw std::domain_error("FBO::attachTexture - one of the dimensions is zero");
	}

	if (int(m_tex_id.size()) == m_max_color_attachments) {
		throw std::out_of_range("FBO::attachTexture - GL_MAX_COLOR_ATTACHMENTS exceeded");
	}

	attachment = GL_COLOR_ATTACHMENT0 + static_cast<GLenum>(m_tex_id.size()); // common attachment for color textures
	if (iformat == GL_RGBA16F_ARB || iformat == GL_RGBA32F_ARB) {
		format = GL_RGBA;
		type = GL_FLOAT;
	}
	else if (iformat == GL_RGB16F_ARB || iformat == GL_RGB32F_ARB) {
		format = GL_RGB;
		type = GL_FLOAT;
	}
	else if (iformat == GL_LUMINANCE_ALPHA16F_ARB || iformat == GL_LUMINANCE_ALPHA32F_ARB) {
		format = GL_LUMINANCE_ALPHA;
		type = GL_FLOAT;
	}
	else if (iformat == GL_LUMINANCE16F_ARB || iformat == GL_LUMINANCE32F_ARB) {
		format = GL_LUMINANCE;
		type = GL_FLOAT;
	}
	else if (iformat == GL_RGBA8 || iformat == GL_RGBA || iformat == 4) {
		format = GL_RGBA;
		type = GL_UNSIGNED_BYTE;
	}
	else if (iformat == GL_RGB8 || iformat == GL_RGB || iformat == 3) {
		format = GL_RGB;
		type = GL_UNSIGNED_BYTE;
	}
	else if (iformat == GL_LUMINANCE8_ALPHA8 || iformat == GL_LUMINANCE_ALPHA || iformat == 2) {
		format = GL_LUMINANCE_ALPHA;
		type = GL_UNSIGNED_BYTE;
	}
	else if (iformat == GL_LUMINANCE8 || iformat == GL_LUMINANCE16 || iformat == GL_LUMINANCE || iformat == 1) {
		format = GL_LUMINANCE;
		type = GL_UNSIGNED_BYTE;
	}
	else if (iformat == GL_DEPTH_COMPONENT24 || iformat == GL_DEPTH_COMPONENT) {
		format = GL_DEPTH_COMPONENT;
		type = GL_UNSIGNED_INT;
		attachment = GL_DEPTH_ATTACHMENT;
		filter = GL_NEAREST;
	}
	else if (iformat == GL_STENCIL_INDEX1 || iformat == GL_STENCIL_INDEX4 || iformat == GL_STENCIL_INDEX8 ||
		iformat == GL_STENCIL_INDEX16 || iformat == GL_STENCIL_INDEX) {
		format = GL_STENCIL_INDEX;
		type = GL_UNSIGNED_BYTE;
		attachment = GL_STENCIL_ATTACHMENT;
		filter = GL_NEAREST;
	}
	else if (iformat == GL_DEPTH24_STENCIL8 || iformat == GL_DEPTH_STENCIL) {
		format = GL_DEPTH_STENCIL;
		type = GL_UNSIGNED_INT_24_8;
		attachment = GL_DEPTH_STENCIL_ATTACHMENT;
		filter = GL_NEAREST;
	}
	else {
		throw std::invalid_argument("FBO::AttachTexture - unrecognized internal format");
	}

	glGenTextures(1, &tex_id);
	glBindFramebuffer(GL_FRAMEBUFFER, m_frame_id);
	glBindTexture(GL_TEXTURE_2D, tex_id);
	glTexImage2D(GL_TEXTURE_2D, 0, iformat, m_width, m_height, 0, format, type, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
	if (format == GL_DEPTH_COMPONENT) {
		glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
	}
	if (format == GL_DEPTH_STENCIL) { // packed depth and stencil added separately
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, tex_id, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, GL_TEXTURE_2D, tex_id, 0);
	}
	else {
		glFramebufferTexture2D(GL_FRAMEBUFFER, attachment, GL_TEXTURE_2D, tex_id, 0);
	}

	m_tex_id.push_back(tex_id);
	m_buffers[m_tex_id.size() - 1] = attachment;
}

inline void FBO::bindInput() {
	for (int i = 0; i < int(m_tex_id.size()); i++) {
		glActiveTexture(GL_TEXTURE0 + i);
		glBindTexture(GL_TEXTURE_2D, m_tex_id[i]);
	}
}

inline void FBO::bindOutput() {
	if (m_tex_id.empty()) {
		throw std::domain_error("FBO::BindIn - no textures to bind");
	}	

	glBindFramebuffer(GL_FRAMEBUFFER, m_frame_id);
	if (m_tex_id.size() == 1) {
		glDrawBuffer(m_buffers[0]);
	}
	else {
		glDrawBuffers(static_cast<GLsizei>(m_tex_id.size()), m_buffers);
	}
}

inline void FBO::bindTex(int num) {
	if (num + 1 > int(m_tex_id.size())) {
		throw std::out_of_range("FBO::BindTex - texture vector size exceeded");
	}

	glBindTexture(GL_TEXTURE_2D, m_tex_id[num]);
}

inline void FBO::blitTo(FBO *dest, GLbitfield mask, GLenum filter) {
	int old_read, old_draw;

	glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &old_read);
	glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &old_draw);;

	if ((mask & GL_DEPTH_BUFFER_BIT) || (mask & GL_STENCIL_BUFFER_BIT)) {
		filter = GL_NEAREST;
	}

	glBindFramebuffer(GL_READ_FRAMEBUFFER, m_frame_id);
	if (dest)
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, dest->m_frame_id);
	else
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

	glBlitFramebuffer(0, 0, m_width, m_height, 0, 0, m_width, m_height, mask, filter);

	glBindFramebuffer(GL_READ_FRAMEBUFFER, old_read);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, old_draw);
}

inline void FBO::check() {
	GLenum status;

	glBindFramebuffer(GL_FRAMEBUFFER, m_frame_id);
	status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE) {
		std::cout << "FBO Status error: " << status << std::endl;
		throw std::invalid_argument("FBO::check - status error");
	}
}

inline void FBO::unbind() {
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glDrawBuffer(GL_BACK);
}
