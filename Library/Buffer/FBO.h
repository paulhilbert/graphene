/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef FBO_H
#define FBO_H

#include <include/common.h>
#include <include/ogl.h>

namespace Buffer {

class FBO {
	public:
		typedef std::shared_ptr<FBO> Ptr;
		typedef std::weak_ptr<FBO>   WPtr;

	public:
		/// Default Constructor
		FBO();
		/// Constructor used when size is known
		FBO(int width, int height);
		/// Destructor
		~FBO();
		/// Set FBO size when using default constructor
		void SetSize(int width, int height) { m_width = width; m_height = height; };
		/// Attach a render target to the FBO
		void AttachRender(GLenum iformat);
		/// Attach a texture to the FBO
		void AttachTexture(GLenum iformat, GLint filter = GL_LINEAR);
		/// Bind the FBO as input, for reading from
		void BindInput();
		/// Bind the FBO as output, for writing into
		void BindOutput();
		/// Bind the specified FBO texture to the context
		void BindTex(int num = 0);
		/// Blit from an FBO to another
		void BlitTo(FBO *dest, GLbitfield mask, GLenum filter = GL_LINEAR);
		/// Check OpenGL status of the FBO
		void Check();

		/// Disable rendering to FBO
		static void Unbind();

	protected:
		int m_width;
		int m_height;
		GLuint m_frame_id;
		GLuint m_depth_id;
		GLuint m_stencil_id;
		std::vector<GLuint> m_tex_id;
		GLenum* m_buffers; // glDrawBuffers() needs a C array

		int m_max_color_attachments;
};


} // Buffer

#endif
