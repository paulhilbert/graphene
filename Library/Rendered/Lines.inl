/// LINERENDERKERNEL ///

inline LineRenderKernel::LineRenderKernel(int lineWidth) : RenderKernel(), m_lineWidth(lineWidth) {
}

inline LineRenderKernel::~LineRenderKernel() {
}

inline void LineRenderKernel::initShader() {
	m_prog.addShaders("Library/GLSL/points.vert", "Library/GLSL/points.frag");
}

inline void LineRenderKernel::renderElements(int pointCount) {
	glLineWidth(m_lineWidth);
	glDrawArrays(GL_LINES, 0, pointCount);
	glLineWidth(1);
}

/// RENDEREDLINES ///

inline Lines::Lines(Annotation::Color baseColor, int lineWidth) : Base(baseColor, RenderKernel::Ptr(new LineRenderKernel(lineWidth))) {
}

inline Lines::~Lines() {
}
