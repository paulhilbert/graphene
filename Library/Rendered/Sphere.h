#ifndef RENDEREDSPHERE_H_
#define RENDEREDSPHERE_H_

#include <Eigen/Dense>
using Eigen::Vector3f;
using Eigen::Vector4f;

#include "../Buffer/Geometry.h"
#include "../Shader/ShaderProgram.h"
using Shader::ShaderProgram;

namespace Rendered {

class Sphere {
	public:
		typedef std::shared_ptr<Sphere> Ptr;
		typedef std::weak_ptr<Sphere> WPtr;
		typedef std::shared_ptr<const Sphere> ConstPtr;
		typedef std::weak_ptr<const Sphere> ConstWPtr;

	public:
		Sphere(Vector3f position, float radius, Vector4f color, int rings = 16, int segments = 16);
		virtual ~Sphere();

		void moveTo(Vector3f position);
		void setRadius(float radius);
		void setColor(Vector4f color);

		void render(ShaderProgram& program);

	protected:
		void generate();

	protected:
		Vector3f m_position;
		float m_radius;
		Vector4f m_color;
		int m_rings;
		int m_segments;
		Buffer::Geometry::Ptr m_geometry;

		unsigned int m_vCount;
		unsigned int m_iCount;
};


} // Rendered

#endif /* RENDEREDSPHERE_H_ */
