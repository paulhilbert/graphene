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
	setPosition(position);
	setLineWidth(lineWidth);

	m_prog.addShaders(std::string(GLSL_PREFIX)+"translatedPoints.vert", std::string(GLSL_PREFIX)+"constant.frag");
	m_prog.link();
	setColor(color);

	vector<Eigen::Vector3f> vertices;
	const float size = 0.1f;
	vertices.push_back(Eigen::Vector3f(0.f,  0.f, -size));
	vertices.push_back(Eigen::Vector3f(0.f,  0.f,  size));
	vertices.push_back(Eigen::Vector3f(0.f, -size, 0.f));
	vertices.push_back(Eigen::Vector3f(0.f,  size, 0.f));
	vertices.push_back(Eigen::Vector3f( size, 0.f, 0.f));
	vertices.push_back(Eigen::Vector3f(-size, 0.f, 0.f));

	m_geom.init();
	m_geom.setVertices(vertices);
	m_geom.enableVertices();
	m_geom.upload();
	m_geom.bindVertices(m_prog, "position");
}

void Point::setPosition(const Eigen::Vector3f& position) {
	m_position = position;
}

void Point::setColor(const Eigen::Vector4f& color) {
	m_color = color;

	m_prog.use();
	m_prog.setUniformVec4("color", color.data());
}

void Point::setLineWidth(int lineWidth) {
	m_lineWidth = lineWidth;
}

void Point::render(const Eigen::Matrix4f& mvMatrix, const Eigen::Matrix4f& prMatrix) {
	m_prog.use();

	m_prog.setUniformMat4("mvM", mvMatrix.data());
	m_prog.setUniformMat4("prM", prMatrix.data());
	m_prog.setUniformVec3("translation", m_position.data());

	glLineWidth(static_cast<GLfloat>(m_lineWidth));

	m_geom.bind();
	glDrawArrays(GL_LINES, 0, 6);
	m_geom.release();

	glLineWidth(1);
}

}
