/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


inline Geometry::Geometry() :
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

inline Geometry::~Geometry() {
}

inline void Geometry::init() {
	m_vao.init();
}

inline void Geometry::enableVertices() {
	m_hasVertices = true;
}

inline void Geometry::enableNormals() {
	m_hasNormals = true;
}

inline void Geometry::enableColors() {
	m_hasColors = true;
}

inline void Geometry::enableTexCoords() {
	m_hasTexCoords = true;
}

inline void Geometry::enableIndices() {
	m_hasIndices = true;
}

inline void Geometry::disableVertices() {
	m_hasVertices = false;
}

inline void Geometry::disableNormals() {
	m_hasNormals = false;
}

inline void Geometry::disableColors() {
	m_hasColors = false;
}

inline void Geometry::disableTexCoords() {
	m_hasTexCoords = false;
}

inline void Geometry::disableIndices() {
	m_hasIndices = false;
}

inline void Geometry::bindVertices(ShaderProgram& program, const GLchar* varName) {
	checkInitVertices();
	int idx = getBufferIndex(VERTICES);
	if (idx < 0) return;
	program.bindAttrib((GLuint)idx, varName);
}

inline void Geometry::bindNormals(ShaderProgram& program, const GLchar* varName) {
	checkInitNormals();
	int idx = getBufferIndex(NORMALS);
	if (idx < 0) return;
	program.bindAttrib((GLuint)idx, varName);
}

inline void Geometry::bindColors(ShaderProgram& program, const GLchar* varName) {
	checkInitColors();
	int idx = getBufferIndex(COLORS);
	if (idx < 0) return;
	program.bindAttrib((GLuint)idx, varName);
}

inline void Geometry::bindTexCoords(ShaderProgram& program, const GLchar* varName) {
	checkInitTexCoords();
	int idx = getBufferIndex(TEXCOORDS);
	if (idx < 0) return;
	program.bindAttrib((GLuint)idx, varName);
}

inline bool Geometry::hasVertices() const {
	return m_hasVertices;
}

inline bool Geometry::hasNormals() const {
	return m_hasNormals;
}

inline bool Geometry::hasColors() const {
	return m_hasColors;
}

inline bool Geometry::hasTexCoords() const {
	return m_hasTexCoords;
}

inline bool Geometry::hasIndices() const {
	return m_hasIndices;
}

inline void Geometry::setVertices(const std::vector<Eigen::Vector3f>& vertices) {
	checkInitVertices();
	m_vbo.set(vertices);
	m_hasDataVertices = true;
}

inline void Geometry::setNormals(const std::vector<Eigen::Vector3f>& normals) {
	checkInitNormals();
	m_nbo.set(normals);
	m_hasDataNormals = true;
}

inline void Geometry::setColors(const std::vector<Eigen::Vector4f>& colors) {
	checkInitColors();
	m_cbo.set(colors);
	m_hasDataColors = true;
}

inline void Geometry::setTexCoords(const std::vector<Eigen::Vector2f>& texCoords) {
	checkInitTexCoords();
	m_tbo.set(texCoords);
	m_hasDataTexCoords = true;
}

inline void Geometry::setIndices(const std::vector<GLuint>& indices) {
	checkInitIndices();
	m_ibo.set(indices);
	m_hasDataIndices = true;
}

inline void Geometry::addVertices(const std::vector<Eigen::Vector3f>& vertices) {
	if (!m_hasDataVertices) {
		setVertices(vertices);
		return;
	}
	m_vbo.add(vertices);
}

inline void Geometry::addNormals(const std::vector<Eigen::Vector3f>& normals) {
	if (!m_hasDataNormals) {
		setNormals(normals);
		return;
	}
	m_nbo.add(normals);
}

inline void Geometry::addColors(const std::vector<Eigen::Vector4f>& colors) {
	if (!m_hasDataColors) {
		setColors(colors);
		return;
	}
	m_cbo.add(colors);
}

inline void Geometry::upload(std::set<Buffers> dynamicAccess) {
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

inline void Geometry::uploadVertices(AccessMethod method) {
	if (!m_hasDataVertices) {
		std::cout << "Trying to upload vertices without data! Skipping." << std::endl;
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

inline void Geometry::uploadNormals(AccessMethod method) {
	if (!m_hasDataNormals) {
		std::cout << "Trying to upload normals without data! Skipping." << std::endl;
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

inline void Geometry::uploadColors(AccessMethod method) {
	if (!m_hasDataColors) {
		std::cout << "Trying to upload colors without data! Skipping." << std::endl;
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

inline void Geometry::uploadTexCoords(AccessMethod method) {
	if (!m_hasDataTexCoords) {
		std::cout << "Trying to upload tex coords without data! Skipping." << std::endl;
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

inline void Geometry::uploadIndices(AccessMethod method) {
	if (!m_hasDataIndices) {
		std::cout << "Trying to upload indices without data! Skipping." << std::endl;
		return;
	}
	m_ibo.upload(method);
}

inline GLuint Geometry::getVBOId() const {
	return m_vbo.getId();
}

inline GLuint Geometry::getNBOId() const {
	return m_nbo.getId();
}

inline GLuint Geometry::getCBOId() const {
	return m_cbo.getId();
}

inline GLuint Geometry::getTBOId() const {
	return m_tbo.getId();
}

inline GLuint Geometry::getIBOId() const {
	return m_ibo.getId();
}

inline unsigned int Geometry::getVBOSize() const {
	return m_vbo.getDataSize();
}

inline unsigned int Geometry::getNBOSize() const {
	return m_nbo.getDataSize();
}

inline unsigned int Geometry::getCBOSize() const {
	return m_cbo.getDataSize();
}

inline unsigned int Geometry::getTBOSize() const {
	return m_tbo.getDataSize();
}

inline unsigned int Geometry::getIBOSize() const {
	return m_ibo.getDataSize();
}

inline void Geometry::checkInitVertices() {
	if (m_initVertices) return;
	m_vbo.init();
	m_initVertices = true;
}

inline void Geometry::checkInitNormals() {
	if (m_initNormals) return;
	m_nbo.init();
	m_initNormals = true;
}

inline void Geometry::checkInitColors() {
	if (m_initColors) return;
	m_cbo.init();
	m_initColors = true;
}

inline void Geometry::checkInitTexCoords() {
	if (m_initTexCoords) return;
	m_tbo.init();
	m_initTexCoords = true;
}

inline void Geometry::checkInitIndices() {
	if (m_initIndices) return;
	m_ibo.init();
	m_initIndices = true;
}

inline int Geometry::getBufferIndex(Buffers buffer) {
	if (m_bufferIndexMap.find(buffer) == m_bufferIndexMap.end()) {
		std::cout << "Trying to get buffer index for uninitialized buffer." << std::endl;
		return -1;
	}
	return (int)m_bufferIndexMap[buffer];
}

inline void Geometry::bind() {
	m_vao.bind();
	if (m_hasVertices) m_vao.enableAttrib(getBufferIndex(VERTICES));
	if (m_hasNormals) m_vao.enableAttrib(getBufferIndex(NORMALS));
	if (m_hasColors) m_vao.enableAttrib(getBufferIndex(COLORS));
	if (m_hasTexCoords) m_vao.enableAttrib(getBufferIndex(TEXCOORDS));
	if (m_hasIndices) m_ibo.bind();
}

inline void Geometry::release() {
	if (m_hasVertices) m_vao.disableAttrib(getBufferIndex(VERTICES));
	if (m_hasNormals) m_vao.disableAttrib(getBufferIndex(NORMALS));
	if (m_hasColors) m_vao.disableAttrib(getBufferIndex(COLORS));
	if (m_hasTexCoords) m_vao.disableAttrib(getBufferIndex(TEXCOORDS));
	if (m_hasIndices) m_ibo.release();
	m_vao.release();
}
