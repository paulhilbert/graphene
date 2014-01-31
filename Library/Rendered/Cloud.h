/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef RENDEREDCLOUD_H_
#define RENDEREDCLOUD_H_

#include <include/config.h>
#include "Field.h"

namespace Rendered {

class CloudRenderKernel : public RenderKernel {
	public:
		typedef std::shared_ptr<CloudRenderKernel> Ptr;
		typedef std::weak_ptr<CloudRenderKernel>   WPtr;

	public:
		CloudRenderKernel(int pointSize);
		virtual ~CloudRenderKernel();

		void renderElements(int pointCount) override;

		void setThickness(int thickness);

	protected:
		int m_pointSize;
};

class Cloud : public Field {
	public:
		typedef std::shared_ptr<Cloud> Ptr;
		typedef std::weak_ptr<Cloud>   WPtr;
		typedef Field Base;

	public:
		Cloud(RGBA baseColor, int pointSize = 2);
		virtual ~Cloud();

		template <class InputIterator>
		void setFromPCLCloud(InputIterator first, InputIterator last, std::vector<RGBA>* colors = nullptr, bool ignoreNormals = false);
};

#include "Cloud.inl"

} // Rendered

#endif /* RENDEREDCLOUD_H_ */
