#include "Sphere.h"

namespace Rendered {

Sphere::Sphere(Vector3f position, float radius, Vector4f color, int rings, int segments) : m_position(position), m_radius(radius), m_color(color), m_rings(rings), m_segments(segments) {
	generate();
}

Sphere::~Sphere() {
}

void Sphere::moveTo(Vector3f position) {
	m_position = position;
	generate();
}

void Sphere::setRadius(float radius) {
	m_radius = radius;
	generate();
}

void Sphere::setColor(Vector4f color) {
	if (!m_geometry) return;
	m_color = color;
	std::vector<Vector4f> colors(m_vCount, color);
	m_geometry->setColors(colors);
}

void Sphere::render(ShaderProgram& program) {
	if (!m_geometry) return;
	//m_geometry->bindVertices(program, "position");
	//m_geometry->bindNormals(program, "normal");
	//m_geometry->bindColors(program, "color");

	// render
	m_geometry->bind();
	glDrawElements(GL_TRIANGLES, m_iCount, GL_UNSIGNED_INT, nullptr);
	//glDrawArrays(GL_POINTS, 0, m_vCount);
	m_geometry->release();
}

void Sphere::generate() {
	m_vCount = (m_rings + 1) * (m_segments + 1);
	m_iCount = 6 * m_rings * (m_segments + 1);
	std::vector<Vector3f> pos(m_vCount), nrm(m_vCount);
	std::vector<unsigned int> indices(m_iCount);

	float dRingAngle = static_cast<float>(M_PI / m_rings);
	float dSegmentAngle = static_cast<float>(2.0 * M_PI / m_segments);

	int vertex = 0, index = 0, i = 0;
	for (int ring = 0; ring <= m_rings; ++ring) {
		float r = m_radius * sinf(ring * dRingAngle);
		float y = m_radius * cosf(ring * dRingAngle);
		for (int seg = 0; seg <= m_segments; ++seg) {
			float x = r * sinf(seg * dSegmentAngle);
			float z = r * cosf(seg * dSegmentAngle);

			pos[vertex] = m_position + Vector3f(x, y, z);
			nrm[vertex] = Vector3f(x, y, z).normalized();
			++vertex;

			if (ring != m_rings) {
				indices[index++] = i + m_segments + 1;
				indices[index++] = i;
				indices[index++] = i + m_segments;
				indices[index++] = i + m_segments + 1;
				indices[index++] = i + 1;
				indices[index++] = i;
				++i;
			}
		}
	}
	std::vector<Vector4f> colors(m_vCount, m_color);

	m_geometry.reset();
	m_geometry = std::make_shared<Buffer::Geometry>();
	m_geometry->init();
	m_geometry->addVertices(pos);
	m_geometry->addNormals(nrm);
	m_geometry->addColors(colors);
	m_geometry->setIndices(indices);
	m_geometry->upload();
	m_geometry->enableVertices();
	m_geometry->enableNormals();
	m_geometry->enableColors();
	m_geometry->enableIndices();
}


} // Rendered
