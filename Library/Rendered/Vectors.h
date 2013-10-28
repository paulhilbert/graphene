/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef RENDEREDVECTORS_H_
#define RENDEREDVECTORS_H_

#include <include/config.h>
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
		Vectors(RGBA baseColor, int lineWidth = 1);
		virtual ~Vectors();

		template <class InputIterator>
		void setFromPCLCloudNormals(InputIterator first, InputIterator last, float factor = 1.f);
};

#include "Vectors.inl"

} // Rendered

#endif /* RENDEREDVECTORS_H_ */
