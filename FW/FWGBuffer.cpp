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

	m_position = Texture::Ptr(new Texture(GL_RGB32F, width, height, (GLfloat*)nullptr));
	m_position->setFiltering(GL_NEAREST);
	m_position->bindToRenderbuffer(GL_COLOR_ATTACHMENT0);

	m_color = Texture::Ptr(new Texture(GL_RGBA32F, width, height, (GLfloat*)nullptr));
	m_color->setFiltering(GL_NEAREST);
	m_color->bindToRenderbuffer(GL_COLOR_ATTACHMENT1);

	m_normal = Texture::Ptr(new Texture(GL_RGB32F, width, height, (GLfloat*)nullptr));
	m_normal->setFiltering(GL_NEAREST);
	m_normal->bindToRenderbuffer(GL_COLOR_ATTACHMENT2);

	m_depth = Texture::Ptr(new Texture(GL_DEPTH_COMPONENT32F, width, height, (GLfloat*)nullptr));
	m_depth->bindToRenderbuffer(GL_DEPTH_ATTACHMENT);

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

GLuint GBuffer::framebuffer() {
	return m_fbo;
}

Texture::Ptr GBuffer::position() {
	return m_position;
}

Texture::Ptr GBuffer::color() {
	return m_color;
}

Texture::Ptr GBuffer::normal() {
	return m_normal;
}

void GBuffer::bindWrite() {
	if (!m_initialized) throw std::runtime_error("Trying to bind uninitialized GBuffer");
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_fbo);
}

void GBuffer::bindRead(Texture::Ptr diffuse, Texture::Ptr specular) {
	if (!m_initialized) throw std::runtime_error("Trying to bind uninitialized GBuffer");
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
	glActiveTexture(GL_TEXTURE0);
	m_position->bind();
	glActiveTexture(GL_TEXTURE1);
	m_color->bind();
	glActiveTexture(GL_TEXTURE2);
	m_normal->bind();
	glActiveTexture(GL_TEXTURE3);
	diffuse->bind();
	glActiveTexture(GL_TEXTURE4);
	specular->bind();
}

void GBuffer::release() {
	if (!m_initialized) throw std::runtime_error("Trying to release uninitialized GBuffer");
}

void GBuffer::blitTo(GLuint attachment, GLsizei x, GLsizei y, GLsizei w, GLsizei h) {
	glReadBuffer(attachment);
	glBlitFramebuffer(0, 0, m_width, m_height, x, y, w, h, GL_COLOR_BUFFER_BIT, GL_LINEAR);
}

void GBuffer::clearBuffers() {
	m_position.reset();
	m_color.reset();
	m_normal.reset();
	m_depth.reset();
}

} // FW
