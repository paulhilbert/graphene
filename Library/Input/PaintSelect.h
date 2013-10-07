#ifndef PAINTSELECT_H_
#define PAINTSELECT_H_

#include <memory>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/OpenGL>
#include <Library/Buffer/Geometry.h>
#include <Library/Shader/ShaderProgram.h>

#include "SelectionMethod.h"

namespace Input {

class PaintSelect : public SelectionMethod {
	public:
		typedef std::shared_ptr<PaintSelect> Ptr;
		typedef std::weak_ptr<PaintSelect> WPtr;

	public:
		PaintSelect(FW::VisualizerHandle::Ptr handle, unsigned int radius, unsigned int subdivisions = 300);
		virtual ~PaintSelect();

		void init(const Eigen::Vector4f& color);
		void render();

		void setRadius(unsigned int radius);

		bool pointInSelection(Eigen::Vector3f point);

	protected:
		void uploadCircle(const Eigen::Vector4f& color);

	protected:
		unsigned int m_radius;
		unsigned int m_subdivisions;
		unsigned int m_numPoints;
		int m_currX;
		int m_currY;
		Buffer::Geometry m_geomCircle;
		Shader::ShaderProgram m_progCircle;
};

#include "PaintSelect.inl"

} // Input

#endif /* PAINTSELECT_H_ */
