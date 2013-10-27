#ifndef TEXTURE_H
#define TEXTURE_H

#include <include/common.h>
#include <include/ogl.h>

namespace Buffer {

class Texture {
	public:
		typedef std::shared_ptr<Texture> Ptr;
		typedef std::weak_ptr<Texture>   WPtr;

	private:
		GLuint m_id;
		void Load(GLenum iformat, int width, int height, GLfloat *pixels);
		void Load(GLenum iformat, int width, int height, GLubyte *pixels);
	public:
		Texture();
		Texture(GLenum iformat, int width, int height, GLfloat *pixels);
		Texture(GLenum iformat, int width, int height, GLubyte *pixels);
		~Texture();
		/// Set texture filtering for both magnification and minification
		void SetFiltering(GLenum filter);
		/// Bind the texture to the current unit
		void Bind() { glBindTexture(GL_TEXTURE_2D, m_id); };
		/// Disable texture rendering for the current unit
		static void Unbind() { glBindTexture(GL_TEXTURE_2D, 0); };
};

} // Buffer

#endif
