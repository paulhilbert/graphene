/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */

#include <include/config.h>

#include "Cube.h"

#include <algorithm>
using std::transform;
#include <vector>
using std::vector;

namespace Rendered {

Cube::Cube(const Eigen::Affine3f& trafo, const Eigen::Vector4f& diffuseColor, bool noHDRShading) {
	vector<Eigen::Vector3f> vertices;

	const float size = 0.5f;

	vertices.push_back(Eigen::Vector3f( size,  size,  size));
	vertices.push_back(Eigen::Vector3f( size,  size, -size));
	vertices.push_back(Eigen::Vector3f(-size,  size,  size));
	vertices.push_back(Eigen::Vector3f(-size,  size, -size));

	vertices.push_back(Eigen::Vector3f(-size, -size, -size));
	vertices.push_back(Eigen::Vector3f( size, -size, -size));
	vertices.push_back(Eigen::Vector3f( size, -size,  size));
	vertices.push_back(Eigen::Vector3f(-size, -size,  size));

	vector<Eigen::Vector3f> normals(vertices.size(), Eigen::Vector3f::Zero());
	if (!noHDRShading) {
		transform(vertices.begin(), vertices.end(), normals.begin(), [] (const Eigen::Vector3f& v) {return v.normalized();});
	}

	for (auto& v : vertices) {
		v = trafo * v;
	}

	vector<unsigned int> indices(14);
	indices[ 0] = 0;
	indices[ 1] = 1;
	indices[ 2] = 2;
	indices[ 3] = 3;
	indices[ 4] = 4;
	indices[ 5] = 1;
	indices[ 6] = 5;
	indices[ 7] = 6;
	indices[ 8] = 4;
	indices[ 9] = 7;
	indices[10] = 2;
	indices[11] = 6;
	indices[12] = 0;
	indices[13] = 1;
	
	std::vector<Eigen::Vector4f> colors(indices.size(), diffuseColor);

	m_geom.init();
	m_geom.setVertices(vertices);
	m_geom.setNormals(normals);
	m_geom.setColors(colors);
	m_geom.setIndices(indices);
	m_geom.enableVertices();
	m_geom.enableNormals();
	m_geom.enableIndices();
	m_geom.enableColors();
	m_geom.upload();
}

void Cube::render(ShaderProgram& program) {
	m_geom.bindVertices(program, "position");
	m_geom.bindNormals(program, "normal");
	m_geom.bindColors(program, "color");

	m_geom.bind();
	glDrawElements(GL_TRIANGLE_STRIP, 14, GL_UNSIGNED_INT, (char*)NULL);
	m_geom.release();
}

}
