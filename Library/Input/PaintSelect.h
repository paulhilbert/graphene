/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef PAINTSELECT_H_
#define PAINTSELECT_H_

#include <include/common.h>

#include <Color/Types.h>
using Color::RGBA;

#include <Library/Buffer/Geometry.h>
#include <Library/Shader/ShaderProgram.h>

#include "SelectionMethod.h"

namespace Input {

class PaintSelect : public SelectionMethod {
	public:
		typedef std::shared_ptr<PaintSelect> Ptr;
		typedef std::weak_ptr<PaintSelect> WPtr;

	public:
		PaintSelect(FW::VisualizerHandle::Ptr handle, unsigned int radius = 10, const RGBA& color = RGBA(0.f, 0.4f, 1.f, 0.4f), unsigned int subdivisions = 300);
		virtual ~PaintSelect();

		void init();
		void render();

		void setRadius(unsigned int radius);
		void setColor(const RGBA& color);

		bool pointInSelection(Eigen::Vector3f point);

	protected:
		void uploadCircle();

	protected:
		unsigned int    m_radius;
		Eigen::Vector4f m_color;
		unsigned int    m_subdivisions;
		unsigned int    m_numPoints;
		int m_currX;
		int m_currY;
		Buffer::Geometry m_geomCircle;
		Shader::ShaderProgram m_progCircle;
};

#include "PaintSelect.inl"

} // Input

#endif /* PAINTSELECT_H_ */
