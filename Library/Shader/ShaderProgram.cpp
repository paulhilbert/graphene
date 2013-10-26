/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "ShaderProgram.h"

#include <string>

namespace Shader {


ShaderProgram::ShaderProgram() : m_ref(0), m_linkStatus(0) {
}

void ShaderProgram::addShaders(std::string vertexShader, std::string fragmentShader, std::string geometryShader) {
	addShader(VShaderPtr(VertexShader::fromFile(vertexShader)));
	addShader(FShaderPtr(FragmentShader::fromFile(fragmentShader)));
	if (geometryShader != "") addShader(GShaderPtr(GeometryShader::fromFile(geometryShader)));
}

void ShaderProgram::link() {
	// create program
	m_ref = glCreateProgram();
	// attach shaders
	glAttachShader(m_ref, m_vShader->getReference());
	glAttachShader(m_ref, m_fShader->getReference());
	if (m_gShader) glAttachShader(m_ref, m_gShader->getReference());
	// link
	glLinkProgram(m_ref);
	glGetProgramiv(m_ref, GL_LINK_STATUS, &m_linkStatus);
	printProgramInfoLog();

	if (!m_linkStatus) std::cout << "Unable to link program" << std::endl;
}

void ShaderProgram::use() {
	glUseProgram(m_ref);
}

void ShaderProgram::bindAttrib(GLuint pos, const GLchar* name) {
	glBindAttribLocation(m_ref, pos, name);
}

void ShaderProgram::setUniformVar1f(const GLchar* name, float value) {
	GLint location = getUniformVarLocation(name);
	glUniform1f(location, value);
}

void ShaderProgram::setUniformVar1f(const GLchar* name, GLsizei count, const float* values) {
	GLint location = getUniformVarLocation(name);
	glUniform1fv(location, count, values);
}

void ShaderProgram::setUniformVar1i(const GLchar* name, int value) {
	GLint location = getUniformVarLocation(name);
	glUniform1i(location, value);
}

void ShaderProgram::setUniformVar1i(const GLchar* name, GLsizei count, const int* values) {
	GLint location = getUniformVarLocation(name);
	glUniform1iv(location, count, values);
}

void ShaderProgram::setUniformVar1b(const GLchar* name, bool value) {
	setUniformVar1i(name, static_cast<int>(value));
}

void ShaderProgram::setUniformVec2(const GLchar* name, const float* values) {
	GLint location = getUniformVarLocation(name);
	glUniform2fv(location, 1, values);
}

void ShaderProgram::setUniformVec3(const GLchar* name, const float* values) {
	GLint location = getUniformVarLocation(name);
	glUniform3fv(location, 1, values);
}

void ShaderProgram::setUniformVec4(const GLchar* name, const float* values) {
	GLint location = getUniformVarLocation(name);
	glUniform4fv(location, 1, values);
}

void ShaderProgram::setUniformMat2(const GLchar* name, const float* values) {
	GLint location = getUniformVarLocation(name);
	glUniformMatrix2fv(location, 1, false, values);
}

void ShaderProgram::setUniformMat3(const GLchar* name, const float* values) {
	GLint location = getUniformVarLocation(name);
	glUniformMatrix3fv(location, 1, false, values);
}

void ShaderProgram::setUniformMat4(const GLchar* name, const float* values) {
	GLint location = getUniformVarLocation(name);
	glUniformMatrix4fv(location, 1, false, values);
}

bool ShaderProgram::isLinked() {
	return static_cast<bool>(m_linkStatus != 0);
}

GLint ShaderProgram::getUniformVarLocation(const GLchar* name) {
	GLint loc;

	loc = glGetUniformLocation(m_ref, name);

	if (loc == -1) std::cout << "No such uniform named "+std::string(name)+". Maybe this variable is not used inside any shader code." << std::endl;

	return loc;
}

void ShaderProgram::printProgramInfoLog() {
	int infologLength = 0;
	int charsWritten  = 0;
	GLchar *infoLog;

	glGetProgramiv(m_ref, GL_INFO_LOG_LENGTH, &infologLength);

	if (infologLength > 1) {
		infoLog = (GLchar *)malloc(infologLength);
		if (infoLog == NULL) {
			std::cout << "Could not allocate InfoLog buffer" << std::endl;
			return;
		}
		glGetProgramInfoLog(m_ref, infologLength, &charsWritten, infoLog);
		std::cout << "Program InfoLog:\n"+std::string(infoLog) << std::endl;
		free(infoLog);
	}
}


} // Shader
