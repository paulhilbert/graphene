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
