#ifndef RENDEREDSPHERES_H_
#define RENDEREDSPHERES_H_

#include "Field.h"
#include "Sphere.h"

namespace Rendered {


class Spheres : public Field {
	public:
		typedef std::shared_ptr<Spheres> Ptr;
		typedef std::weak_ptr<Spheres>   WPtr;

	public:
		Spheres(RGBA baseColor);
		virtual ~Spheres();

		void set(const std::vector<Eigen::Vector3f>& points, float radius, const std::vector<RGBA>* colors = nullptr);

		void render(ShaderProgram& program) override;

		void setThickness(int thickness) { /* nop */ }

	protected:
		void set(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>* normals = nullptr, const std::vector<RGBA>* colors = nullptr) { /* nop */ }
		void setColors(const Annotation::Colors& colors);
		void upload();

	protected:
		std::vector<Sphere> m_spheres;
		GeometryPtr         m_geometry;
		unsigned int        m_pointCount;
};

} // Rendered

#endif /* RENDEREDSPHERES_H_ */
