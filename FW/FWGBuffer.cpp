#include "FWGBuffer.h"

#include <stdexcept>

namespace FW {


GBuffer::GBuffer() : m_initialized(false) {
	m_clearProg.addShaders(std::string(GLSL_PREFIX) + "fullQuad.vert", std::string(GLSL_PREFIX) + "clearBuffers.frag");
	std::map<int, std::string>  outputMap;
	outputMap[0] = "outPos";
	outputMap[1] = "outCol";
	outputMap[2] = "outNrm";
	m_clearProg.link(outputMap);

	std::vector<Vector3f>  quadVertices;
	quadVertices.push_back(Vector3f(-1.f, -1.f, 0.f));
	quadVertices.push_back(Vector3f( 1.f, -1.f, 0.f));
	quadVertices.push_back(Vector3f( 1.f,  1.f, 0.f));
	quadVertices.push_back(Vector3f(-1.f,  1.f, 0.f));
	std::vector<GLuint>  quadIndices(6);
	quadIndices[0] = 0; quadIndices[1] = 1; quadIndices[2] = 2;
	quadIndices[3] = 0; quadIndices[4] = 2; quadIndices[5] = 3;
	m_geomQuad.init();
	m_geomQuad.setVertices(quadVertices);
	m_geomQuad.setIndices(quadIndices);
	m_geomQuad.upload();
	m_geomQuad.enableVertices();
	m_geomQuad.enableIndices();
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

	m_blur = Texture::Ptr(new Texture(GL_RGB32F, width, height, (GLfloat*)nullptr));
	m_blur->bindToRenderbuffer(GL_COLOR_ATTACHMENT3);

	m_bloom = Texture::Ptr(new Texture(GL_RGB32F, width, height, (GLfloat*)nullptr));
	m_bloom->bindToRenderbuffer(GL_COLOR_ATTACHMENT4);

	m_occlusion = Texture::Ptr(new Texture(GL_RGB32F, width, height, (GLfloat*)nullptr));
	m_occlusion->bindToRenderbuffer(GL_COLOR_ATTACHMENT5);

	m_ssao = Texture::Ptr(new Texture(GL_RGB32F, width, height, (GLfloat*)nullptr));
	m_ssao->bindToRenderbuffer(GL_COLOR_ATTACHMENT6);

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

void GBuffer::bindGeomPass(ShaderProgram& program, const Eigen::Vector4f& clearColor, Texture::Ptr diffuse, Texture::Ptr specular) {
	if (!m_initialized) throw std::runtime_error("Trying to bind uninitialized GBuffer");
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_fbo);
	GLenum buffers[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
	glDrawBuffers(3, buffers);

	clearBuffers(clearColor);
	glActiveTexture(GL_TEXTURE0);
	diffuse->bind();
	glActiveTexture(GL_TEXTURE1);
	specular->bind();

	program.use();
	program.setTexture("mapDiff", 0);
	program.setTexture("mapSpec", 1);
}

void GBuffer::bindLightPass(ShaderProgram& program) {
	if (!m_initialized) throw std::runtime_error("Trying to bind uninitialized GBuffer");
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
	glActiveTexture(GL_TEXTURE0);
	m_color->bind();
	glActiveTexture(GL_TEXTURE1);
	m_depth->bind();
	glActiveTexture(GL_TEXTURE2);
	m_blur->bind();
	glActiveTexture(GL_TEXTURE3);
	m_bloom->bind();
	glActiveTexture(GL_TEXTURE4);
	m_ssao->bind();

	program.use();
	program.setTexture("mapCol", 0);
	program.setTexture("mapDepth", 1);
	program.setTexture("mapBlur", 2);
	program.setTexture("mapBloom", 3);
	program.setTexture("mapSSAO", 4);
}

void GBuffer::bindBlurPass(ShaderProgram& program) {
	if (!m_initialized) throw std::runtime_error("Trying to bind uninitialized GBuffer");
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_fbo);
	GLenum buffers[] = {GL_COLOR_ATTACHMENT3, GL_COLOR_ATTACHMENT4, GL_COLOR_ATTACHMENT6};
	glDrawBuffers(3, buffers);

	glActiveTexture(GL_TEXTURE0);
	m_color->bind();
	glActiveTexture(GL_TEXTURE1);
	m_occlusion->bind();

	program.use();
	program.setTexture("mapCol", 0);
	program.setTexture("mapOcclusion", 1);
}

void GBuffer::bindSSAOPass(ShaderProgram& program) {
	if (!m_initialized) throw std::runtime_error("Trying to bind uninitialized GBuffer");
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_fbo);
	GLenum buffers[] = {GL_COLOR_ATTACHMENT5};
	glDrawBuffers(1, buffers);

	glActiveTexture(GL_TEXTURE0);
	m_position->bind();
	glActiveTexture(GL_TEXTURE1);
	m_normal->bind();
	//glActiveTexture(GL_TEXTURE2);
	//m_depth->bind();

	program.use();
	program.setTexture("mapPos", 0);
	program.setTexture("mapNrm", 1);
	//program.setTexture("mapDepth", 2);
}

void GBuffer::release() {
	if (!m_initialized) throw std::runtime_error("Trying to release uninitialized GBuffer");
}

void GBuffer::clearBuffers(const Eigen::Vector4f& clearColor) {
	m_clearProg.use();
	m_clearProg.setUniformVec4("clearColor", clearColor.data());

	// store blend mode and disable blending
	GLboolean blendEnabled;
	glGetBooleanv(GL_BLEND, &blendEnabled);
	glDisable(GL_BLEND);

	glViewport(0, 0, m_width, m_height);
	m_geomQuad.bind();
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, (char *)NULL);
	m_geomQuad.release();

	// restore blend mode
	if (blendEnabled) glEnable(GL_BLEND);
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
