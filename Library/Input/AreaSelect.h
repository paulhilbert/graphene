#ifndef AREASELECT_H_
#define AREASELECT_H_

#include <memory>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/OpenGL>
#include <Library/Buffer/Geometry.h>
#include <Library/Shader/ShaderProgram.h>

#include "SelectionMethod.h"

namespace Input {

class AreaSelect : public SelectionMethod {
	public:
		typedef std::shared_ptr<AreaSelect> Ptr;
		typedef std::weak_ptr<AreaSelect> WPtr;

	public:
		AreaSelect(FW::VisualizerHandle::Ptr handle);
		virtual ~AreaSelect();

		void init(const Eigen::Vector4f& color);
		void render();

		bool pointInSelection(Eigen::Vector3f point);

	protected:
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
