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
	m_prog.addShaders("Library/GLSL/points.vert", "Library/GLSL/points.frag");
}

inline void CloudRenderKernel::renderElements(int pointCount) {
	glEnable(GL_DEPTH_TEST);
	glPointSize(static_cast<GLfloat>(m_pointSize));
	glDrawArrays(GL_POINTS, 0, pointCount);
	glPointSize(1);
}

/// RENDEREDCLOUD ///

inline Cloud::Cloud(Annotation::Color baseColor, int pointSize) : Base(baseColor, RenderKernel::Ptr(new CloudRenderKernel(pointSize))) {
}

inline Cloud::~Cloud() {
}

template <class InputIterator>
inline void Cloud::setFromPCLCloud(InputIterator first, InputIterator last) {
	std::vector<Eigen::Vector3f> points(std::distance(first, last));
	std::transform(first, last, points.begin(), [&](const typename InputIterator::value_type& point) { return point.getVector3fMap(); });
	this->set(points);
}
