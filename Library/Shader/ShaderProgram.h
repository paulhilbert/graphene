/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef SHADERPROGRAM_H_
#define SHADERPROGRAM_H_

#include <iostream>
#include <map>
#include <memory>
#include <vector>
using std::vector;

#include <boost/optional.hpp>
#include <boost/none.hpp>
using boost::optional;
using boost::none;

#include "Shader.h"
#include "../Buffer/Texture.h"
using Buffer::Texture;

namespace Shader {

class ShaderProgram {
	public:
		typedef std::shared_ptr<ShaderProgram>   Ptr;
		typedef std::weak_ptr<ShaderProgram>   WPtr;
		typedef std::shared_ptr<const ShaderProgram>   ConstPtr;
		typedef std::weak_ptr<const ShaderProgram>   ConstWPtr;

		typedef std::shared_ptr<VertexShader>    VShaderPtr;
		typedef std::shared_ptr<FragmentShader>  FShaderPtr;
		typedef std::shared_ptr<GeometryShader>  GShaderPtr;
	public:
		ShaderProgram();

		template <int ShaderType>
		void addShader(std::shared_ptr< Shader<ShaderType> > shader);
		void addShaders(std::string vertexShader, std::string fragmentShader, std::string geometryShader = "");

		void link(optional<std::map<int, std::string>> outputMap = none);
		void use();

		void bindAttrib(GLuint pos, const GLchar* name);
		void setUniformVar1f(const GLchar* name, float value);
		void setUniformVar1f(const GLchar* name, GLsizei count, const float* values);
		void setUniformVar1i(const GLchar* name, int value);
		void setUniformVar1i(const GLchar* name, GLsizei count, const int* values);
		void setUniformVar1b(const GLchar* name, bool value);
		void setUniformVec2(const GLchar* name, const float* values);
		void setUniformVec3(const GLchar* name, const float* values);
		void setUniformVec4(const GLchar* name, const float* values);
		void setUniformMat2(const GLchar* name, const float* values);
		void setUniformMat3(const GLchar* name, const float* values);
		void setUniformMat4(const GLchar* name, const float* values);
		void setTexture(const GLchar* name, int unit);

		bool isLinked();

	protected:
		GLint getUniformVarLocation(const GLchar* name);

		void printProgramInfoLog();

	protected:
		GLuint      m_ref;
		VShaderPtr  m_vShader;
		FShaderPtr  m_fShader;
		GShaderPtr  m_gShader;
		GLint       m_linkStatus;
};


template <int ShaderType>
inline void ShaderProgram::addShader(std::shared_ptr< Shader<ShaderType> > shader) {
	switch (ShaderType) {
		case GL_VERTEX_SHADER:   m_vShader = std::dynamic_pointer_cast<VertexShader>(shader); break;
		case GL_FRAGMENT_SHADER: m_fShader = std::dynamic_pointer_cast<FragmentShader>(shader); break;
		case GL_GEOMETRY_SHADER: m_gShader = std::dynamic_pointer_cast<GeometryShader>(shader); break;
		default: std::cout << "Unknown shader type" << std::endl;
	}
}


} // Shader

#endif /* SHADERPROGRAM_H_ */
