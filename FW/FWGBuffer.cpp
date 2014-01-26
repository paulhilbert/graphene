#include "FWGBuffer.h"

#include <stdexcept>

namespace FW {


GBuffer::GBuffer() : m_initialized(false) {
}

GBuffer::GBuffer(int width, int height) : m_initialized(false) {
	init(width, height);
}

GBuffer::~GBuffer() {
	clearBuffers();
}

void GBuffer::init(int width, int height) {
	if (m_initialized) clearBuffers();

	m_width = width;
	m_height = height;

	glGenFramebuffers(1, &m_fbo);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_fbo);

	glGenTextures(NUM_TEX, m_tex);
	glGenTextures(1, &m_depth);

	for (unsigned int i=0; i < NUM_TEX; ++i) {
		glBindTexture(GL_TEXTURE_2D, m_tex[i]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, width, height, 0, GL_RGB, GL_FLOAT, NULL);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + i, GL_TEXTURE_2D, m_tex[i], 0);
	}

	glBindTexture(GL_TEXTURE_2D, m_depth);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
	glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_depth, 0);

	GLenum buffers[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
	glDrawBuffers(3, buffers);

	auto status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE) throw std::runtime_error("Incomplete/incorrect framebuffer setup");

	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

	m_initialized = true;
}

bool GBuffer::initialized() const {
	return m_initialized;
}

void GBuffer::bindWrite() {
	if (!m_initialized) throw std::runtime_error("Trying to bind uninitialized GBuffer");
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_fbo);
}

void GBuffer::bindRead() {
	if (!m_initialized) throw std::runtime_error("Trying to bind uninitialized GBuffer");
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
	for (unsigned int i = 0 ; i < NUM_TEX; i++) {
		glActiveTexture(GL_TEXTURE0 + i);
		glBindTexture(GL_TEXTURE_2D, m_tex[i]);
	}
}

void GBuffer::release() {
	if (!m_initialized) throw std::runtime_error("Trying to release uninitialized GBuffer");
}

void GBuffer::blitTo(TEX_TYPE tex, GLsizei x, GLsizei y, GLsizei w, GLsizei h) {
	glReadBuffer(GL_COLOR_ATTACHMENT0 + tex);
	glBlitFramebuffer(0, 0, m_width, m_height, x, y, w, h, GL_COLOR_BUFFER_BIT, GL_LINEAR);
}

void GBuffer::clearBuffers() {
	glDeleteRenderbuffers(1, &m_depth);
	glDeleteRenderbuffers(NUM_TEX, m_tex);
	glDeleteFramebuffers(1, &m_fbo);
}

} // FW
