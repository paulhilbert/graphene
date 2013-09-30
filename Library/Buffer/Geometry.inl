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
		Log::warn("Trying to get buffer index for uninitialized buffer.");
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
