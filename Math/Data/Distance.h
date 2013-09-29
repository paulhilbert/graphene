#ifndef DISTANCE_H_
#define DISTANCE_H_

#include <vector>
using std::vector;
using std::pair;

namespace Math {
namespace Data {

template <class Scalar, class Point>
class Distance {
	public:
		typedef vector<Point> PointSet;
		class PointPair;

	public:
		Distance(const PointSet& points);
		virtual ~Distance() {}

		virtual Scalar compute(PointPair p);
		unsigned int getPointCount() const;

	protected:
		virtual Scalar computeDistance(const Point& x, const Point& y) = 0;

	protected:
		const PointSet& m_points;

	public:
		class PointPair : public pair<unsigned int, unsigned int> {
			public:
				PointPair(unsigned int p0, unsigned int p1);

				bool operator ==(const PointPair& other) const;
				bool operator !=(const PointPair& other) const;
		};
};

template <class Scalar, class Point>
Distance<Scalar,Point>::Distance(const PointSet& points) : m_points(points) {
}

template <class Scalar, class Point>
Scalar Distance<Scalar,Point>::compute(PointPair p) {
	return computeDistance(m_points[p.first], m_points[p.second]);
}

template <class Scalar, class Point>
inline unsigned int Distance<Scalar,Point>::getPointCount() const {
	return m_points.size();
}

template <class Scalar, class Point>
Distance<Scalar,Point>::PointPair::PointPair(unsigned int p0, unsigned int p1) : pair(p0<p1?p0:p1,p0<p1?p1:p0) {
}

template <class Scalar, class Point>
inline bool Distance<Scalar,Point>::PointPair::operator ==(const PointPair& other) const {
	return (first == other.first && second == other.second || first == other.second && second == other.first);
}

template <class Scalar, class Point>
inline bool Distance<Scalar,Point>::PointPair::operator !=(const PointPair& other) const {
	return !operator ==(other);
}

} // Data
} // Math

#endif /* DISTANCE_H_ */
