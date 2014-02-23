/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */

#include <include/config.h>

#include "Point.h"

namespace Rendered {

#include <vector>
using std::vector;

Point::Point(const Eigen::Vector3f& position, const Eigen::Vector4f& color, int lineWidth) {
	m_position = position;
	m_color = color;
	m_lineWidth = lineWidth;

	updateRenderedLines();
}

void Point::setPosition(const Eigen::Vector3f& position) {
	m_position = position;
	updateRenderedLines();
}

void Point::setColor(const Eigen::Vector4f& color) {
	m_color = color;
	updateRenderedLines();
}

void Point::setLineWidth(int lineWidth) {
	m_lineWidth = lineWidth;
	updateRenderedLines();
}

void Point::updateRenderedLines() {
	m_renderedLines = Lines::Ptr(new Lines(m_color, m_lineWidth));

	vector<Eigen::Vector3f> vertices;
	const float size = 0.1f;
	vertices.push_back(Eigen::Vector3f(0.f,  0.f, -size) + m_position);
	vertices.push_back(Eigen::Vector3f(0.f,  0.f,  size) + m_position);
	vertices.push_back(Eigen::Vector3f(0.f, -size, 0.f) + m_position);
	vertices.push_back(Eigen::Vector3f(0.f,  size, 0.f) + m_position);
	vertices.push_back(Eigen::Vector3f( size, 0.f, 0.f) + m_position);
	vertices.push_back(Eigen::Vector3f(-size, 0.f, 0.f) + m_position);

	m_renderedLines->set(vertices);
}

void Point::render(ShaderProgram& program) {
	m_renderedLines->render(program);
}

}
