#ifndef SHADER_H_
#define SHADER_H_

#include <iostream>
#include <fstream>
using std::ifstream;

#include <cstdio>
#include <cstdlib>

#include <GL/glew.h>

namespace Shader {


template <int ShaderType>
class Shader {
	public:
		static Shader<ShaderType>* fromFile(std::string filename);
		virtual ~Shader();

		GLenum getShaderType();
		GLuint getReference();

	protected:
		Shader();

		void readCode(std::string filename);
		void create();
		void compile();
		void printShaderInfoLog();

	protected:
		GLuint  m_ref;
		GLchar* m_code;
		GLint   m_compileStatus;
		GLint   m_linkStatus;
};


typedef Shader<GL_VERTEX_SHADER>   VertexShader;
typedef Shader<GL_FRAGMENT_SHADER> FragmentShader;
typedef Shader<GL_GEOMETRY_SHADER> GeometryShader;


template <int ShaderType>
Shader<ShaderType>* Shader<ShaderType>::fromFile(std::string filename) {
	if (ShaderType != GL_VERTEX_SHADER && ShaderType != GL_FRAGMENT_SHADER && ShaderType != GL_GEOMETRY_SHADER) {
		std::cout << "Unknown shader type" << std::endl;
		return NULL;
	}
	Shader* result = new Shader();
	
	result->readCode(filename);
	result->create();
	result->compile();

	return result;
}

template <int ShaderType>
Shader<ShaderType>::~Shader() {
	if (m_code) delete [] m_code;
}

template <int ShaderType>
inline GLenum Shader<ShaderType>::getShaderType() {
	return ShaderType;
}
template <int ShaderType>
inline GLuint Shader<ShaderType>::getReference() {
	return m_ref;
}

template <int ShaderType>
Shader<ShaderType>::Shader() : m_ref(0), m_code(NULL), m_compileStatus(0), m_linkStatus(0) {
}

template <int ShaderType>
void Shader<ShaderType>::readCode(std::string filename) {
	std::ifstream f;
	f.open(filename.c_str(), std::ios::binary);
	if(!f.is_open() || !f.good()) {
		std::cout << "Could not open file" << std::endl;
		return;
	}
	std::ifstream::pos_type begin_pos = f.tellg();
	f.seekg(0, std::ios_base::end);
	int vertFileSize = static_cast<int>(f.tellg() - begin_pos);
	f.seekg(0, std::ios_base::beg);
	m_code = new GLchar[vertFileSize + 1];
	m_code[vertFileSize] = 0;
	f.read(m_code, vertFileSize);
	if (!m_code) {
		// no shader read, complain loudly
		std::cout << "No shader read" << std::endl;
	}
}

template <int ShaderType>
void Shader<ShaderType>::create() {
	m_ref = glCreateShader(ShaderType);
	const GLchar* code = m_code;
	glShaderSource(m_ref, 1, &code, NULL);
}

template <int ShaderType>
void Shader<ShaderType>::compile() {
	glCompileShader(m_ref);
	glGetShaderiv(m_ref, GL_COMPILE_STATUS, &m_compileStatus);
	printShaderInfoLog();

	if (!m_compileStatus) {
		std::cout << "Could not compile shader" << std::endl;
	}
}

template <int ShaderType>
void Shader<ShaderType>::printShaderInfoLog() {
	int infologLength = 0;
	int charsWritten  = 0;
	GLchar *infoLog;

	glGetShaderiv(m_ref, GL_INFO_LOG_LENGTH, &infologLength);

	if (infologLength > 1) {
		infoLog = (GLchar *)malloc(infologLength);
		if (infoLog == NULL) {
			std::cout << "ERROR: Could not allocate InfoLog buffer" << std::endl;
			exit(1);
		}
		glGetShaderInfoLog(m_ref, infologLength, &charsWritten, infoLog);
		switch (ShaderType) {
			case GL_VERTEX_SHADER:   std::cout << "Vertex Shader InfoLog:\n"+std::string(infoLog) << std::endl;   break;
			case GL_FRAGMENT_SHADER: std::cout << "Fragment Shader InfoLog:\n"+std::string(infoLog) << std::endl; break;
			case GL_GEOMETRY_SHADER: std::cout << "Geometry Shader InfoLog:\n"+std::string(infoLog) << std::endl; break;
			default: std::cout << "Unknown shader type" << std::endl;
		}
		free(infoLog);
	}
}


} // Shader

#endif /* SHADER_H_ */
