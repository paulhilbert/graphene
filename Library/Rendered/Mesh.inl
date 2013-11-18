template <class MeshType>
Mesh<MeshType>::Mesh(MeshPtr mesh, Shader::ShaderProgram::Ptr program, bool smoothNormals, bool allowSwitching) : m_mesh(mesh), m_program(program), m_smooth(smoothNormals), m_allowSwitching(allowSwitching) {
	m_geometry = GeometryPtr(new Buffer::Geometry());
	m_geometry->init();
	init();
	upload();
	m_geometry->upload();
	m_geometry->enableVertices();
	m_geometry->enableNormals();
	m_geometry->enableColors();
	m_geometry->enableIndices();
	m_geometry->bindVertices(*m_program, "position");
	m_geometry->bindNormals(*m_program, "normal");
	m_geometry->bindColors(*m_program, "color");
}

template <class MeshType>
Mesh<MeshType>::~Mesh() {
}

template <class MeshType>
void Mesh<MeshType>::render() {
	m_geometry->bind();
	glDrawElements(GL_TRIANGLES, 3*Traits::numFaces(*m_mesh), GL_UNSIGNED_INT, nullptr);
	m_geometry->release();
}

template <class MeshType>
void Mesh<MeshType>::setSmoothNormals(bool smoothNormals) {
	if (smoothNormals == m_smooth) return;
	if (!m_allowSwitching) {
		throw std::runtime_error("Rendered::Mesh : Switch normal smoothing disabled. Instantiate with AllowSwitching=true to enable.");
	}
	m_smooth = smoothNormals;
	upload();
}

template <class MeshType>
void Mesh<MeshType>::init() {
	auto faces = Traits::faces(*m_mesh);
	
	if (m_smooth || m_allowSwitching) { // init smooth vertices
		auto points = Traits::vertices(*m_mesh);
		m_smoothVertices.resize(points.size());
		m_smoothNormals.resize(points.size());
		m_smoothColors.resize(points.size());
		int i = 0;
		for (const auto& idx : points) {
			m_smoothVertices[i] = Traits::vertexPosition(*m_mesh, idx);
			m_smoothNormals[i] = Traits::vertexNormal(*m_mesh, idx);
			m_smoothColors[i] = Traits::vertexColor(*m_mesh, idx);
			++i;
		}
		m_smoothIndices.resize(faces.size()*3);
		i = 0;
		for (const auto& face : faces) {
			for (const auto& vertex : Traits::faceVertices(*m_mesh, face)) {
				m_smoothIndices[i++] = vertex;
			}
		}
	}
	
	if (!m_smooth || m_allowSwitching) { // init flat vertices
		m_flatVertices.resize(faces.size()*3);
		m_flatNormals.resize(faces.size()*3);
		m_flatColors.resize(faces.size()*3);
		int i = 0;
		for (const auto& face : faces) {
			Vector3f normal = Traits::faceNormal(*m_mesh, face);
			for (const auto& idx : Traits::faceVertices(*m_mesh, face)) {
				m_flatVertices[i] = Traits::vertexPosition(*m_mesh, idx);
				m_flatNormals[i] = normal;
				m_flatColors[i] = Traits::vertexColor(*m_mesh, idx);
				++i;
			}
		}
		m_flatIndices.resize(faces.size()*3);
		std::iota(m_flatIndices.begin(), m_flatIndices.end(), 0);
	}
}

template <class MeshType>
void Mesh<MeshType>::upload() {
	m_geometry->setVertices(m_smooth ? m_smoothVertices : m_flatVertices);
	m_geometry->setNormals(m_smooth ? m_smoothNormals : m_flatNormals);
	m_geometry->setColors(m_smooth ? m_smoothColors : m_flatColors);
	m_geometry->setIndices(m_smooth ? m_smoothIndices : m_flatIndices);
	m_geometry->upload();
}
