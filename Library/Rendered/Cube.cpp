#include "Cube.h"

#include <algorithm>
using std::transform;
#include <vector>
using std::vector;

namespace Rendered {

Cube::Cube(const Eigen::Affine3f& trafo, const Eigen::Vector4f& ambientColor, const Eigen::Vector4f& diffuseColor) {
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

	vector<Eigen::Vector3f> normals(vertices.size());
	transform(vertices.begin(), vertices.end(), normals.begin(), [] (const Eigen::Vector3f& v) {return v.normalized();});

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

	m_prog.addShaders("GLSL/mesh.vert", "GLSL/mesh.frag");
	m_prog.link();
	m_prog.use();
	m_prog.setUniformVec3("lightDir", Eigen::Vector3f(1.f, 0.5f, 2.f).normalized().data());
	m_prog.setUniformVec3("clipNormal", Eigen::Vector3f(0.f, 0.f, 1.f).data());
	m_prog.setUniformVar1f("clipDistance", 0.f);
	m_prog.setUniformVec4("ambient", ambientColor.data());
	m_prog.setUniformVec4("diffuse", diffuseColor.data());

	m_geom.init();
	m_geom.setVertices(vertices);
	m_geom.setNormals(normals);
	m_geom.setIndices(indices);
	m_geom.enableVertices();
	m_geom.enableNormals();
	m_geom.enableIndices();
	m_geom.upload();
	m_geom.bindVertices(m_prog, "position");
	m_geom.bindNormals(m_prog, "normal");
}

void Cube::render(const Eigen::Matrix4f& mvMatrix, const Eigen::Matrix4f& prMatrix, const Eigen::Matrix3f& nmMatrix) {
	m_prog.use();

	m_prog.setUniformMat4("mvM", mvMatrix.data());
	m_prog.setUniformMat4("prM", prMatrix.data());
	m_prog.setUniformMat3("nmM", nmMatrix.data());

	m_geom.bind();
	glDrawElements(GL_TRIANGLE_STRIP, 14, GL_UNSIGNED_INT, (char*)NULL);
	m_geom.release();
}

}
