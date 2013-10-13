/// VECTORRENDERKERNEL ///

inline VectorRenderKernel::VectorRenderKernel(int lineWidth) : RenderKernel(), m_lineWidth(lineWidth) {
}

inline VectorRenderKernel::~VectorRenderKernel() {
}

inline void VectorRenderKernel::initShader() {
	m_prog.addShaders("Library/GLSL/vectors.vert", "Library/GLSL/vectors.frag", "Library/GLSL/vectors.geom");
}

inline void VectorRenderKernel::renderElements(int pointCount) {
	glLineWidth(m_lineWidth);
	glDrawArrays(GL_LINES, 0, pointCount);
	glLineWidth(1);
}

/// RENDEREDVECTORS ///

inline Vectors::Vectors(Annotation::Color baseColor, int lineWidth) : Base(baseColor, RenderKernel::Ptr(new VectorRenderKernel(lineWidth))) {
}

inline Vectors::~Vectors() {
}

template <class InputIterator>
inline void Vectors::setFromPCLCloudNormals(InputIterator first, InputIterator last, float factor) {
	std::vector<Eigen::Vector3f> normals(2*std::distance(first, last));
	unsigned int idx=0;
	for (; first != last; ++first) {
		normals[idx++] = first->getVector3fMap();
		normals[idx++] = first->getVector3fMap() + factor * first->getNormalVector3fMap();
	}
	this->set(normals);
}
