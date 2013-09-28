#ifndef RENDEREDPOINT_H
#define RENDEREDPOINT_H

#include <Eigen/Dense>

#include "../Buffer/Geometry.h"
#include "../Shader/ShaderProgram.h"

namespace Rendered {

class Point {
	public:
		Point(const Eigen::Vector3f& position = Eigen::Vector3f(0.f, 0.f, 0.f), const Eigen::Vector4f& color = Eigen::Vector4f(1.f, 1.f, 1.f, 1.f), int lineWidth = 1);

		void setPosition(const Eigen::Vector3f& position);
		void setColor(const Eigen::Vector4f& color);
		void setLineWidth(int lineWidth);

		inline const Eigen::Vector3f& getPosition() {return m_position;}
		inline const Eigen::Vector4f& getColor() {return m_color;}
		inline int getLineWidth() {return m_lineWidth;}

		void render(const Eigen::Matrix4f& mvMatrix, const Eigen::Matrix4f& prMatrix);

	protected:
		Eigen::Vector3f m_position;
		Eigen::Vector4f m_color;
		int m_lineWidth;

		Buffer::Geometry        m_geom;
		Shader::ShaderProgram   m_prog;
};

}

#endif // RENDEREDPOINT_H
