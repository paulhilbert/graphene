/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


/// CLOUDRENDERKERNEL ///

inline CloudRenderKernel::CloudRenderKernel(int pointSize) : RenderKernel(), m_pointSize(pointSize) {
}

inline CloudRenderKernel::~CloudRenderKernel() {
}

inline void CloudRenderKernel::initShader() {
	m_prog.addShaders(std::string(GLSL_PREFIX)+"simpleLighting.vert", std::string(GLSL_PREFIX)+"simpleLighting.frag");
	m_progHDR.addShaders(std::string(GLSL_PREFIX)+"hdrLighting.vert", std::string(GLSL_PREFIX)+"hdrLighting.frag");
	m_progHDR.link();
	m_progHDR.use();
	m_progHDR.setUniformVar1i("Tex0", 0);
	m_progHDR.setUniformVar1i("Tex1", 1);
}

inline void CloudRenderKernel::renderElements(int pointCount) {
	Eigen::Vector3f lightDir(1.f, 1.f, 1.f);
	lightDir.normalize();
	m_prog.setUniformVec3("lightDir", lightDir.data());
	glEnable(GL_DEPTH_TEST);
//	glEnable(GL_POINT_SMOOTH);
	glPointSize(static_cast<GLfloat>(m_pointSize));
	glDrawArrays(GL_POINTS, 0, pointCount);
//	glDisable(GL_POINT_SMOOTH);
	glPointSize(1);
}

inline void CloudRenderKernel::renderElementsHDR(int pointCount) {
	glEnable(GL_DEPTH_TEST);
//	glEnable(GL_POINT_SMOOTH);
	glPointSize(static_cast<GLfloat>(m_pointSize));
	glDrawArrays(GL_POINTS, 0, pointCount);
//	glDisable(GL_POINT_SMOOTH);
	glPointSize(1);
}

inline void CloudRenderKernel::setThickness(int thickness) {
	m_pointSize = thickness;
}

/// RENDEREDCLOUD ///

inline Cloud::Cloud(RGBA baseColor, int pointSize) : Base(baseColor, RenderKernel::Ptr(new CloudRenderKernel(pointSize))) {
}

inline Cloud::~Cloud() {
}

template <class InputIterator>
inline void Cloud::setFromPCLCloud(InputIterator first, InputIterator last) {
	std::vector<Eigen::Vector3f> points(std::distance(first, last));
	std::vector<Eigen::Vector3f> normals(std::distance(first, last));
	std::transform(first, last, points.begin(), [&](const typename InputIterator::value_type& point) { return point.getVector3fMap(); });
	std::transform(first, last, normals.begin(), [&](const typename InputIterator::value_type& point) { return point.getNormalVector3fMap(); });
	this->set(points, normals);
}
