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
		FBO();
		FBO(int width, int height);
		~FBO();

		void setSize(int width, int height);
		void attachRender(GLenum iformat);
		void attachTexture(GLenum iformat, GLint filter = GL_LINEAR);
		void bindInput();
		void bindOutput();
		void bindTex(int num = 0);
		void blitTo(FBO *dest, GLbitfield mask, GLenum filter = GL_LINEAR);
		void check();

		static void unbind();

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

#include "FBO.inl"

} // Buffer

#endif
