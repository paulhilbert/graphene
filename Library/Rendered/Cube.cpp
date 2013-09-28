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

	vector<unsigned int> indices({0, 1, 2, 3, 4, 1, 5, 6, 4, 7, 2, 6, 0, 1});

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
