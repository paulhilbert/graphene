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
