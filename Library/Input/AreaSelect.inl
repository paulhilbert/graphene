/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


inline AreaSelect::AreaSelect(FW::VisualizerHandle::Ptr handle, const RGBA& color) : SelectionMethod(handle), m_color(color) {
}

inline AreaSelect::~AreaSelect() {
}

inline void AreaSelect::init() {
	m_progArea.addShaders("Library/GLSL/points.vert", "Library/GLSL/points.frag");
	m_progArea.link();
	m_geomArea.init();
	uploadColors();

	m_handle->events()->connect<void (int, int)>("LEFT_DRAG_START", [&] (int x, int y) {
		if (!m_enabled) return;
		m_startX = x;
		m_startY = y;
		if (m_start) m_start();
	});
	m_handle->events()->connect<void (int, int, int, int)>("LEFT_DRAG", [&] (int dx, int dy, int x, int y) {
		if (!m_enabled) return;
		m_currX = x;
		m_currY = y;
		m_area.setEmpty();
		m_area.extend(Eigen::Vector2i(m_startX, m_startY));
		m_area.extend(Eigen::Vector2i(m_currX, m_currY));
		Eigen::Vector3f min, max, c0, c1;
		min << m_area.min().cast<float>(), 0.f;
		max << m_area.max().cast<float>(), 0.f;
		c0 << max[0], min[1], 0.f;
		c1 << min[0], max[1], 0.f;
		m_geomArea.setVertices({ min, c0, max, c1 });
		m_geomArea.upload({Buffer::Geometry::VERTICES});
		m_geomArea.enableColors();
		m_geomArea.enableVertices();
		m_geomArea.bindVertices(m_progArea, "position");
		m_geomArea.bindColors(m_progArea, "icolor");
		m_dragging = true;
		if (m_drag) m_drag();
	});
	m_handle->events()->connect<void (int, int)>("LEFT_DRAG_STOP", [&] (int x, int y) {
		if (!m_enabled) return;
		m_dragging = false;
		if (m_stop) {
			m_stop();
		}
	});
	m_handle->events()->connect<void (int, int)>("LEFT_CLICK", [&] (int x, int y) {
		if (!m_enabled) return;
		if (m_unselect) m_unselect();
	});
}

inline void AreaSelect::setColor(const RGBA& color) {
	m_color = color;
	uploadColors();
}

inline void AreaSelect::render() {
	if (!m_dragging) return;
	Eigen::Matrix4f mv = Eigen::Matrix4f::Identity();
	auto vp = m_handle->transforms()->viewport();
	auto pr = Eigen::ortho(vp[0], vp[2], vp[1], vp[3]);
	m_progArea.use();
	m_progArea.setUniformMat4("mvM", mv.data());
	m_progArea.setUniformMat4("prM", pr.data());
	GLboolean blendEnabled;
	glGetBooleanv(GL_BLEND, &blendEnabled);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	m_geomArea.bind();
	glDrawArrays(GL_QUADS, 0, 4);
	m_geomArea.release();

	if (!blendEnabled) glDisable(GL_BLEND);
}

inline bool AreaSelect::pointInSelection(Eigen::Vector3f point) {
	auto proj = Eigen::project(point, m_handle->transforms()->modelview(), m_handle->transforms()->projection(), m_handle->transforms()->viewport());
	return m_area.contains(proj.head(2).cast<int>());
}

inline void AreaSelect::uploadColors() {
	m_geomArea.setColors({m_color, m_color, m_color, m_color});
}
