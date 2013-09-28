#ifndef RENDEREDCLOUD_H_
#define RENDEREDCLOUD_H_

#include "Field.h"

namespace Rendered {

class CloudRenderKernel : public RenderKernel {
	public:
		typedef std::shared_ptr<CloudRenderKernel> Ptr;
		typedef std::weak_ptr<CloudRenderKernel>   WPtr;

	public:
		CloudRenderKernel(int pointSize);
		virtual ~CloudRenderKernel();

		void initShader() override;
		void renderElements(int pointCount) override;

	protected:
		int m_pointSize;
};

class Cloud : public Field {
	public:
		typedef std::shared_ptr<Cloud> Ptr;
		typedef std::weak_ptr<Cloud>   WPtr;
		typedef Field Base;

	public:
		Cloud(Annotation::Color baseColor, int pointSize = 2);
		virtual ~Cloud();

		template <class InputIterator>
		void setFromPCLCloud(InputIterator first, InputIterator last);
};

#include "Cloud.inl"

} // Rendered

#endif /* RENDEREDCLOUD_H_ */
