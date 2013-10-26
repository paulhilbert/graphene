/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef RENDEREDLINES_H
#define RENDEREDLINES_H

#include "Field.h"


namespace Rendered {

class LineRenderKernel : public RenderKernel {
	public:
		typedef std::shared_ptr<LineRenderKernel> Ptr;
		typedef std::weak_ptr<LineRenderKernel>   WPtr;

	public:
		LineRenderKernel(int lineWidth);
		virtual ~LineRenderKernel();

		void initShader() override;
		void renderElements(int pointCount) override;

	protected:
		int m_lineWidth;
};

class Lines : public Field {
	public:
		typedef std::shared_ptr<Lines> Ptr;
		typedef std::weak_ptr<Lines>   WPtr;
		typedef Field Base;

	public:
		Lines(Annotation::Color baseColor, int lineWidth = 1);
		virtual ~Lines();
};

#include "Lines.inl"

} // Rendered

#endif // RENDEREDLINES_H
