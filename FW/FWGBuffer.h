#ifndef FWGBUFFER_H_
#define FWGBUFFER_H_

#include <memory>
#include <Eigen/Dense>

#include <include/ogl.h>
#include <include/config.h>

#include <Library/Buffer/Texture.h>
#include <Library/Buffer/Geometry.h>
#include <Library/Shader/ShaderProgram.h>
using Buffer::Texture;
using Shader::ShaderProgram;

namespace FW {

class GBuffer {
	public:
		typedef std::shared_ptr<GBuffer>        Ptr;
		typedef std::weak_ptr<GBuffer>          WPtr;
		typedef std::shared_ptr<const GBuffer>  ConstPtr;
		typedef std::weak_ptr<const GBuffer>    ConstWPtr;

	public:
		GBuffer();
		GBuffer(int width, int height);
		virtual ~GBuffer();

		void init(int width, int height);

		bool initialized() const;

		GLuint       framebuffer();
		Texture::Ptr position();
		Texture::Ptr color();
		Texture::Ptr normal();

		void bindGeomPass(ShaderProgram& program, const Eigen::Vector4f& clearColor, Texture::Ptr diffuse, Texture::Ptr specular);
		void bindLightPass(ShaderProgram& program);
		void bindPostHPass(ShaderProgram& program);
		void bindPostVPass(ShaderProgram& program);
		void release();

		void clearBuffers(const Eigen::Vector4f& clearColor);

		void blitTo(GLuint attachment, GLsizei x, GLsizei y, GLsizei w, GLsizei h);

	protected:
		void clearBuffers();

	protected:
		int   m_width;
		int   m_height;
		bool  m_initialized;

		GLuint  m_fbo;
		Texture::Ptr m_position;
		Texture::Ptr m_color;
		Texture::Ptr m_normal;
		Texture::Ptr m_depth;

		Texture::Ptr m_blur;
		Texture::Ptr m_bloom;
		Texture::Ptr m_aux0;
		Texture::Ptr m_aux1;

		ShaderProgram m_clearProg;
		Buffer::Geometry m_geomQuad;
};

} // FW

#endif /* FWGBUFFER_H_ */
