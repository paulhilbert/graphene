#ifndef TEXTURE_H
#define TEXTURE_H

#include <include/common.h>
#include <include/ogl.h>

namespace Buffer {

class Texture {
	public:
		typedef std::shared_ptr<Texture>        Ptr;
		typedef std::weak_ptr<Texture>          WPtr;
		typedef std::shared_ptr<const Texture>  ConstPtr;
		typedef std::weak_ptr<const Texture>    ConstWPtr;

	public:
		Texture();
		Texture(GLenum iformat, int width, int height, GLfloat *pixels);
		Texture(GLenum iformat, int width, int height, GLubyte *pixels);
		~Texture();

		void setFiltering(GLenum filter);
		void bind();
		static void unbind();

		void bindToRenderbuffer(GLuint attachment);

		GLuint id() const;

	protected:
		void load(GLenum iformat, int width, int height, GLfloat *pixels);
		void load(GLenum iformat, int width, int height, GLubyte *pixels);

	protected:
		GLuint m_id;
		bool   m_boundToRB;
};

#include "Texture.inl"

} // Buffer

#endif
