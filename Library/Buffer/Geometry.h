#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <map>
#include <set>
using std::map;

#include <Eigen/Dense>

#include <Vis/Color.h>

#include "../Shader/ShaderProgram.h"
using Shader::ShaderProgram;

#include "VAO.h"
#include "VBO.h"
#include "IBO.h"

namespace Buffer {


class Geometry {
	public:
		typedef VBO<GLfloat,2> VBO2f;
		typedef VBO<GLfloat,3> VBO3f;
		typedef VBO<GLfloat,4> VBO4f;
		typedef enum {VERTICES, NORMALS, COLORS, TEXCOORDS, INDICES} Buffers;

	public:
		Geometry();
		virtual ~Geometry();

		void init();

		void enableVertices();
		void enableNormals();
		void enableColors();
		void enableTexCoords();
		void enableIndices();

		void disableVertices();
		void disableNormals();
		void disableColors();
		void disableTexCoords();
		void disableIndices();

		void bindVertices(ShaderProgram& program, const GLchar* varName);
		void bindNormals(ShaderProgram& program, const GLchar* varName);
		void bindColors(ShaderProgram& program, const GLchar* varName);
		void bindTexCoords(ShaderProgram& program, const GLchar* varName);

		bool hasVertices()  const;
		bool hasNormals()   const;
		bool hasColors()    const;
		bool hasTexCoords() const;
		bool hasIndices()   const;

		void setVertices (const std::vector<Eigen::Vector3f>& vertices);
		void setNormals  (const std::vector<Eigen::Vector3f>& normals);
		void setColors   (const std::vector<Eigen::Vector4f>& colors);
		void setTexCoords(const std::vector<Eigen::Vector2f>& texCoords);
		void setIndices  (const std::vector<GLuint>& indices);

		void addVertices (const std::vector<Eigen::Vector3f>& vertices);
		void addColors   (const std::vector<Eigen::Vector4f>& colors);

		void upload(std::set<Buffers> dynamicAccess = std::set<Buffers>());
		void uploadVertices(AccessMethod method = STATIC_ACCESS);
		void uploadNormals(AccessMethod method = STATIC_ACCESS);
		void uploadColors(AccessMethod method = STATIC_ACCESS);
		void uploadTexCoords(AccessMethod method = STATIC_ACCESS);
		void uploadIndices(AccessMethod method = STATIC_ACCESS);

		GLuint getVBOId() const;
		GLuint getNBOId() const;
		GLuint getCBOId() const;
		GLuint getTBOId() const;
		GLuint getIBOId() const;

		unsigned int getVBOSize() const;
		unsigned int getNBOSize() const;
		unsigned int getCBOSize() const;
		unsigned int getTBOSize() const;
		unsigned int getIBOSize() const;

		void bind();
		void release();

	protected:
		void checkInitVertices();
		void checkInitNormals();
		void checkInitColors();
		void checkInitTexCoords();
		void checkInitIndices();

		int getBufferIndex(Buffers buffer);

	protected:
		VAO    m_vao;
		VBO3f  m_vbo;
		VBO3f  m_nbo;
		VBO4f  m_cbo;
		VBO2f  m_tbo;
		IBO    m_ibo;

		bool   m_hasVertices;
		bool   m_hasNormals;
		bool   m_hasColors;
		bool   m_hasTexCoords;
		bool   m_hasIndices;

		bool   m_initVertices;
		bool   m_initNormals;
		bool   m_initColors;
		bool   m_initTexCoords;
		bool   m_initIndices;

		bool   m_hasDataVertices;
		bool   m_hasDataNormals;
		bool   m_hasDataColors;
		bool   m_hasDataTexCoords;
		bool   m_hasDataIndices;

		map<Buffers, GLuint> m_bufferIndexMap;
};


#include "Geometry.inl"

} // Buffer

#endif /* GEOMETRY_H_ */
