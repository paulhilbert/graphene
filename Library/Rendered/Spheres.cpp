#include "Spheres.h"

namespace Rendered {

Spheres::Spheres(RGBA baseColor) : Field(baseColor, nullptr) {
}

Spheres::~Spheres() {
}

void Spheres::set(const std::vector<Eigen::Vector3f>& points, float radius, const std::vector<RGBA>* colors) {
	m_pointCount = static_cast<unsigned int>(points.size());

	m_baseColors.clear();
	if (colors) {
		if (colors->size() == m_pointCount) {
			m_baseColors = *colors;
		} else {
			throw std::runtime_error("Mismatch of point and color count.");
		}
	} else {
		m_baseColors = std::vector<RGBA>(m_pointCount, m_color);
	}

	m_colors = m_baseColors;
	m_spheres.clear();
	for (unsigned int i=0; i<m_pointCount; ++i) {
		m_spheres.emplace_back(points[i], radius, m_colors[i]);
	}
}

void Spheres::render(ShaderProgram& program) {
	if (!m_visible || !m_spheres.size()) return;
	for (auto& s : m_spheres) {
		s.render(program);
	}
}

void Spheres::upload() {
	for (unsigned int i=0; i<m_pointCount; ++i) {
		m_spheres[i].setColor(m_colors[i]);
	}
}

} // Rendered
