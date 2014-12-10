inline NormalField::NormalField(RGBA baseColor) : m_color(baseColor), m_pointCount(0) {
}

inline NormalField::~NormalField() {
}

inline void NormalField::set(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>* normals, const std::vector<RGBA>* colors) {
	m_pointCount = static_cast<unsigned int>(points.size());

	m_geometry.reset();
	m_geometry = std::shared_ptr<Buffer::Geometry>(new Buffer::Geometry());
	m_geometry->init();
	m_geometry->addVertices(points);

	if (normals) {
		if (normals->size() == m_pointCount) {
			m_geometry->addNormals(*normals);
		} else {
			throw std::runtime_error("Mismatch of point and normal count.");
		}
	} else {
		std::vector<Eigen::Vector3f> nrms(m_pointCount, Eigen::Vector3f::Zero());
		m_geometry->addNormals(nrms);
	}

	if (colors) {
		if (colors->size() == m_pointCount) {
			m_geometry->addColors(*colors);
		} else {
			throw std::runtime_error("Mismatch of point and color count.");
		}
	} else {
		std::vector<RGBA> cols(m_pointCount, m_color);
		m_geometry->addColors(cols);
	}

	m_geometry->upload();
	m_geometry->enableVertices();
	m_geometry->enableNormals();
	m_geometry->enableColors();

}

template <typename Func>
inline void NormalField::render(ShaderProgram& program, Func&& renderCall) {
	if (!m_visible || !m_geometry) return;
	m_geometry->bindVertices(program, "position");
	m_geometry->bindNormals(program, "normals");
	m_geometry->bindColors(program, "color");

	// store blend mode and enable blending
	//GLboolean blendEnabled;
	//glGetBooleanv(GL_BLEND, &blendEnabled);
	//glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// render
	m_geometry->bind();
	renderCall(m_pointCount);
	m_geometry->release();

	// restore blend mode
	//if (!blendEnabled) glDisable(GL_BLEND);
}
