#ifndef RENDEREDVECTORS_H_
#define RENDEREDVECTORS_H_

#include "Field.h"

namespace Rendered {

class VectorRenderKernel : public RenderKernel {
	public:
		typedef std::shared_ptr<VectorRenderKernel> Ptr;
		typedef std::weak_ptr<VectorRenderKernel>   WPtr;

	public:
		VectorRenderKernel(int lineWidth);
		virtual ~VectorRenderKernel();

		void initShader() override;
		void renderElements(int pointCount) override;

	protected:
		int m_lineWidth;
};

class Vectors : public Field {
	public:
		typedef std::shared_ptr<Vectors> Ptr;
		typedef std::weak_ptr<Vectors>   WPtr;
		typedef Field Base;

	public:
		Vectors(Annotation::Color baseColor, int lineWidth = 1);
		virtual ~Vectors();

		template <class InputIterator>
		void setFromPCLCloudNormals(InputIterator first, InputIterator last, float factor = 1.f);
};

#include "Vectors.inl"

} // Rendered

#endif /* RENDEREDVECTORS_H_ */
