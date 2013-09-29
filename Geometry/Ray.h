#ifndef GEOMETRYRAY_H_
#define GEOMETRYRAY_H_

#include <tuple>
#include <limits>
#include <memory>
#include <boost/optional.hpp>
#include <boost/none.hpp>
#include <Eigen/Dense>


namespace Geometry {

class Ray {
	public:
		typedef std::shared_ptr<Ray> Ptr;
		typedef std::weak_ptr<Ray>   WPtr;

		typedef Eigen::Vector3f          Vec;
		typedef boost::optional<Vec>     OptVec;
		typedef std::pair<float,float>   Limits;
		typedef std::tuple<Vec,Vec,Vec>  Triangle;

		static constexpr float inf = std::numeric_limits<float>::infinity();

	public:
		Ray(Vec direction = Vec(0,0,1), Vec origin = Vec(0,0,0));

		Vec getOrigin() const;
		Vec getDirection() const;

		Vec eval(float t) const;

		void setOrigin(Vec origin);
		void setDirection(Vec direction);
		void setPoints(Vec p0, Vec p1);

		OptVec intersectRay(const Ray& other) const;
		bool   intersectSphere(const Vec& c, float r, Vec& intersection, float& lambda, Limits limits = {0.f, inf}) const;
		bool   intersectTriangle(const Triangle& t, Vec& intersection, float& lambda, Limits limits = {0.f, inf}) const;
		float  distance( const Ray& other, Limits limitThis = {0.f, inf}, Limits limitOther = {0.f, inf}, Vec* pointThis = NULL, Vec* pointOther = NULL ) const;

	protected:
		Vec  m_direction;
		Vec  m_origin;
};

inline bool operator>>(float t, const Ray::Limits& limits) {
	return (t >= limits.first && t <= limits.second);
}

#include "Ray.inl"

} // Geometry

#endif /* GEOMETRYRAY_H_ */
