/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


/// LINERENDERKERNEL ///

inline LineRenderKernel::LineRenderKernel(int lineWidth) : RenderKernel(), m_lineWidth(lineWidth) {
}

inline LineRenderKernel::~LineRenderKernel() {
}

inline void LineRenderKernel::initShader() {
	m_prog.addShaders(std::string(GLSL_PREFIX)+"points.vert", std::string(GLSL_PREFIX)+"points.frag");
}

inline void LineRenderKernel::renderElements(int pointCount) {
	glLineWidth(m_lineWidth);
	glDrawArrays(GL_LINES, 0, pointCount);
	glLineWidth(1);
}

/// RENDEREDLINES ///

inline Lines::Lines(RGBA baseColor, int lineWidth) : Base(baseColor, RenderKernel::Ptr(new LineRenderKernel(lineWidth))) {
}

inline Lines::~Lines() {
}
