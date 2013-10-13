inline PaintSelect::PaintSelect(FW::VisualizerHandle::Ptr handle, unsigned int radius, const RGBA& color, unsigned int subdivisions) : SelectionMethod(handle), m_radius(radius), m_color(color), m_subdivisions(subdivisions), m_numPoints(subdivisions+2) {
}

inline PaintSelect::~PaintSelect() {
}

inline void PaintSelect::init() {
	m_progCircle.addShaders("Library/GLSL/points.vert", "Library/GLSL/points.frag");
	m_progCircle.link();
	m_geomCircle.init();
	uploadCircle();


	m_handle->events()->connect<void (int, int)>("LEFT_DRAG_START", [&] (int x, int y) {
		if (!m_enabled) return;
		if (m_start) m_start();
	});
	m_handle->events()->connect<void (int, int, int, int)>("LEFT_DRAG", [&] (int dx, int dy, int x, int y) {
		if (!m_enabled) return;
		m_currX = x;
		m_currY = y;
		m_dragging = true;
		if (m_drag) m_drag();
	});
	m_handle->events()->connect<void (int, int)>("LEFT_DRAG_STOP", [&] (int x, int y) {
		if (!m_enabled) return;
		m_dragging = false;
		if (m_stop) m_stop();
	});
	m_handle->events()->connect<void (int, int)>("LEFT_PRESS", [&] (int x, int y) {
		if (!m_enabled) return;
		m_currX = x;
		m_currY = y;
		m_dragging = true;
		if (m_drag) m_drag();
	});
	m_handle->events()->connect<void (int, int)>("LEFT_RELEASE", [&] (int x, int y) {
		if (!m_enabled) return;
		m_dragging = false;
		if (m_stop) m_stop();
	});
	m_handle->events()->connect<void (int, int)>("RIGHT_CLICK", [&] (int x, int y) {
		if (!m_enabled) return;
		if (m_unselect) m_unselect();
	});
}

inline void PaintSelect::render() {
	if (!m_dragging) return;
	Eigen::Matrix4f mv = Eigen::Matrix4f::Identity();
	mv(0, 3) = static_cast<float>(m_currX);
	mv(1, 3) = static_cast<float>(m_currY);
	auto vp = m_handle->transforms()->viewport();
	auto pr = Eigen::ortho(vp[0], vp[2], vp[1], vp[3]);
	m_progCircle.use();
	m_progCircle.setUniformMat4("mvM", mv.data());
	m_progCircle.setUniformMat4("prM", pr.data());
	GLboolean blendEnabled;
	glGetBooleanv(GL_BLEND, &blendEnabled);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	m_geomCircle.bind();
	glDrawArrays(GL_TRIANGLE_FAN, 0, m_numPoints);
	m_geomCircle.release();

	if (!blendEnabled) glDisable(GL_BLEND);
}

inline void PaintSelect::setRadius(unsigned int radius) {
	m_radius = radius;
	uploadCircle();
}

inline void PaintSelect::setColor(const Eigen::Vector4f& color) {
	m_color = color;
	uploadCircle();
}

inline bool PaintSelect::pointInSelection(Eigen::Vector3f point) {
	auto proj = Eigen::project(point, m_handle->transforms()->modelview(), m_handle->transforms()->projection(), m_handle->transforms()->viewport());
	Eigen::Vector2f center(static_cast<float>(m_currX), static_cast<float>(m_currY));
	return (proj.head(2) - center).squaredNorm() < static_cast<float>(m_radius*m_radius);
}

inline void PaintSelect::uploadCircle() {
	std::vector<Eigen::Vector3f> points(m_numPoints);
	std::vector<Eigen::Vector4f> colors(m_numPoints, m_color);
	float radius = static_cast<float>(m_radius);
	int idx = 0;
	points[idx++] = Eigen::Vector3f::Zero();
	for (unsigned int i = 0; i <= m_subdivisions; ++i) {
		double angle = 2.0 * M_PI * static_cast<double>(i%m_subdivisions) / m_subdivisions;
		float x = std::cos(angle);
		float y = std::sin(angle);
		points[idx++] = radius * Eigen::Vector3f(x, y, 0.f);
	}
	m_geomCircle.setVertices(points);
	m_geomCircle.setColors(colors);
	m_geomCircle.upload({Buffer::Geometry::VERTICES});
	m_geomCircle.enableVertices();
	m_geomCircle.enableColors();
	m_geomCircle.bindColors(m_progCircle, "icolor");
	m_geomCircle.bindVertices(m_progCircle, "position");
}
