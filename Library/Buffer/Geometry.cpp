#include "Geometry.h"

#include <set>


namespace Buffer {

Geometry::Geometry() :
	m_hasVertices(false),
	m_hasNormals(false),
	m_hasColors(false),
	m_hasTexCoords(false),
	m_hasIndices(false),
   m_initVertices(false),
	m_initNormals(false),
	m_initColors(false),
	m_initTexCoords(false),
	m_initIndices(false),
   m_hasDataVertices(false),
	m_hasDataNormals(false),
	m_hasDataColors(false),
	m_hasDataTexCoords(false),
	m_hasDataIndices(false)	{
}

Geometry::~Geometry() {
}

void Geometry::init() {
	m_vao.init();
}

void Geometry::setVertices(const std::vector<Eigen::Vector3f>& vertices) {
	checkInitVertices();
	m_vbo.set(vertices);
	m_hasDataVertices = true;
}

void Geometry::setNormals(const std::vector<Eigen::Vector3f>& normals) {
	checkInitNormals();
	m_nbo.set(normals);
	m_hasDataNormals = true;
}

void Geometry::setColors(const std::vector<Eigen::Vector4f>& colors) {
	checkInitColors();
	m_cbo.set(colors);
	m_hasDataColors = true;
}

void Geometry::setTexCoords(const std::vector<Eigen::Vector2f>& texCoords) {
	checkInitTexCoords();
	m_tbo.set(texCoords);
	m_hasDataTexCoords = true;
}

void Geometry::setIndices(const std::vector<GLuint>& indices) {
	checkInitIndices();
	m_ibo.set(indices);
	m_hasDataIndices = true;
}

void Geometry::addVertices(const std::vector<Eigen::Vector3f>& vertices) {
	if (!m_hasDataVertices) {
		setVertices(vertices);
		return;
	}
	m_vbo.add(vertices);
}

void Geometry::addColors(const std::vector<Eigen::Vector4f>& colors) {
	if (!m_hasDataColors) {
		setColors(colors);
		return;
	}
	m_cbo.add(colors);
}

void Geometry::upload(std::set<Buffers> dynamicAccess) {
	std::set<Buffers> dyn(dynamicAccess);
	if (m_hasDataVertices) {
		uploadVertices((dyn.find(VERTICES) == dyn.end()) ? STATIC_ACCESS : DYNAMIC_ACCESS);
	}
	if (m_hasDataNormals) {
		uploadNormals((dyn.find(NORMALS) == dyn.end()) ? STATIC_ACCESS : DYNAMIC_ACCESS);
	}
	if (m_hasDataColors) {
		uploadColors((dyn.find(COLORS) == dyn.end()) ? STATIC_ACCESS : DYNAMIC_ACCESS);
	}
	if (m_hasDataIndices) {
		uploadIndices((dyn.find(INDICES) == dyn.end()) ? STATIC_ACCESS : DYNAMIC_ACCESS);
	}
	if (m_hasTexCoords) {
		uploadTexCoords((dyn.find(TEXCOORDS) == dyn.end()) ? STATIC_ACCESS : DYNAMIC_ACCESS);
	}
}

void Geometry::uploadVertices(AccessMethod method) {
	if (!m_hasDataVertices) {
		Log::warn("Trying to upload vertices without data! Skipping.");
		return;
	}
	m_vbo.upload(method);
	m_vao.bind();
	if (m_bufferIndexMap.find(VERTICES) == m_bufferIndexMap.end()) {
		unsigned int idx = static_cast<unsigned int>(m_bufferIndexMap.size());
		m_bufferIndexMap[VERTICES] = idx;
	}
	m_vao.setAttrib(m_bufferIndexMap[VERTICES],3);
	m_vao.release();
}

void Geometry::uploadNormals(AccessMethod method) {
	if (!m_hasDataNormals) {
		Log::warn("Trying to upload normals without data! Skipping.");
		return;
	}
	m_nbo.upload(method);
	m_vao.bind();
	if (m_bufferIndexMap.find(NORMALS) == m_bufferIndexMap.end()) {
		unsigned int idx = static_cast<unsigned int>(m_bufferIndexMap.size());
		m_bufferIndexMap[NORMALS] = idx;
	}
	m_vao.setAttrib(m_bufferIndexMap[NORMALS],3);
	m_vao.release();
}

void Geometry::uploadColors(AccessMethod method) {
	if (!m_hasDataColors) {
		Log::warn("Trying to upload colors without data! Skipping.");
		return;
	}
	m_cbo.upload(method);
	m_vao.bind();
	if (m_bufferIndexMap.find(COLORS) == m_bufferIndexMap.end()) {
		unsigned int idx = static_cast<unsigned int>(m_bufferIndexMap.size());
		m_bufferIndexMap[COLORS] = idx;
	}
	m_vao.setAttrib(m_bufferIndexMap[COLORS],4);
	m_vao.release();
}

void Geometry::uploadTexCoords(AccessMethod method) {
	if (!m_hasDataTexCoords) {
		Log::warn("Trying to upload tex coords without data! Skipping.");
		return;
	}
	m_tbo.upload(method);
	m_vao.bind();
	if (m_bufferIndexMap.find(TEXCOORDS) == m_bufferIndexMap.end()) {
		unsigned int idx = static_cast<unsigned int>(m_bufferIndexMap.size());
		m_bufferIndexMap[TEXCOORDS] = idx;
	}
	m_vao.setAttrib(m_bufferIndexMap[TEXCOORDS],2);
	m_vao.release();
}

void Geometry::uploadIndices(AccessMethod method) {
	if (!m_hasDataIndices) {
		Log::warn("Trying to upload indices without data! Skipping.");
		return;
	}
	m_ibo.upload(method);
}


} // Buffer
