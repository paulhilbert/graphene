/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef AREASELECT_H_
#define AREASELECT_H_

#include <memory>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/OpenGL>

#include <Vis/Color.h>
using Vis::RGBA;

#include <Library/Buffer/Geometry.h>
#include <Library/Shader/ShaderProgram.h>

#include "SelectionMethod.h"

namespace Input {

class AreaSelect : public SelectionMethod {
	public:
		typedef std::shared_ptr<AreaSelect> Ptr;
		typedef std::weak_ptr<AreaSelect> WPtr;

	public:
		AreaSelect(FW::VisualizerHandle::Ptr handle, const RGBA& color = RGBA(0.f, 0.4f, 1.f, 0.4f));
		virtual ~AreaSelect();

		void init();
		void render();

		void setColor(const RGBA& color);

		bool pointInSelection(Eigen::Vector3f point);

	protected:
		void uploadColors();

	protected:
		RGBA m_color;
		int  m_startX;
		int  m_startY;
		int  m_currX;
		int  m_currY;
		Eigen::AlignedBox<int, 2> m_area;
		Buffer::Geometry m_geomArea;
		Shader::ShaderProgram m_progArea;
};

#include "AreaSelect.inl"

} // Input

#endif /* AREASELECT_H_ */
