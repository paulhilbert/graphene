#ifndef AREASELECT_H_
#define AREASELECT_H_

#include <memory>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/OpenGL>
#include <Library/Buffer/Geometry.h>
#include <Library/Shader/ShaderProgram.h>
#include <FW/FWVisualizerHandle.h>
using namespace Shader;

namespace Input {

class AreaSelect {
	public:
		typedef std::shared_ptr<AreaSelect> Ptr;
		typedef std::weak_ptr<AreaSelect> WPtr;

	public:
		AreaSelect(FW::VisualizerHandle::Ptr handle);
		virtual ~AreaSelect();

		void init(const Eigen::Vector4f& color);
		void render();

		void setStartCallback(std::function<void ()> func);
		void setDragCallback(std::function<void ()> func);
		void setStopCallback(std::function<void ()> func);
		void setUnselectCallback(std::function<void ()> func);

		bool pointInSelection(Eigen::Vector3f point);

	protected:
		FW::VisualizerHandle::Ptr m_handle;
		bool m_dragging;
		int  m_startX;
		int  m_startY;
		int  m_currX;
		int  m_currY;
		Eigen::AlignedBox<int, 2> m_area;
		Buffer::Geometry m_geomArea;
		ShaderProgram m_progArea;

		std::function<void ()> m_start;
		std::function<void ()> m_drag;
		std::function<void ()> m_stop;
		std::function<void ()> m_unselect;
};

#include "AreaSelect.inl"

} // Input

#endif /* AREASELECT_H_ */
