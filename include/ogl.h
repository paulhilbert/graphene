/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef INC_OGL_H_
#define INC_OGL_H_

/**
 *  @file ogl.h
 *
 *  This file includes files commonly included by files using OpenGL commands.
 */

#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES
#endif

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
#include <windows.h>
#endif

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>

#endif /* INC_OGL_H_ */
