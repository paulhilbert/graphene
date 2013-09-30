#ifndef DISTANCEMT_H_
#define DISTANCEMT_H_

#include <vector>
using std::vector;
using std::pair;

namespace Math {
namespace Data {

template <class TScalar, class TPoint>
class DistanceMT {
	public:
		typedef TScalar Scalar;
		typedef TPoint  Point;
		typedef vector<TPoint> PointSet;
		class PointPair;

	public:
		DistanceMT(const PointSet& points);
		virtual ~DistanceMT() {}

		virtual TScalar compute(PointPair p);
		unsigned int getPointCount() const;

	protected:
		virtual TScalar computeDistance(const TPoint& x, const TPoint& y) = 0;

	protected:
		const PointSet& m_points;

	public:
		class PointPair : public pair<unsigned int, unsigned int> {
			public:
				PointPair();
				PointPair(unsigned int p0, unsigned int p1);

				bool operator ==(const PointPair& other) const;
				bool operator !=(const PointPair& other) const;
		};
};

template <class TScalar, class TPoint>
DistanceMT<TScalar,TPoint>::DistanceMT(const PointSet& points) : m_points(points) {
}

template <class TScalar, class TPoint>
TScalar DistanceMT<TScalar,TPoint>::compute(PointPair p) {
	return computeDistance(m_points[p.first], m_points[p.second]);
}

template <class TScalar, class TPoint>
inline unsigned int DistanceMT<TScalar,TPoint>::getPointCount() const {
	return m_points.size();
}

template <class TScalar, class TPoint>
DistanceMT<TScalar,TPoint>::PointPair::PointPair(unsigned int p0, unsigned int p1) : pair(p0<p1?p0:p1,p0<p1?p1:p0) {
}

template <class TScalar, class TPoint>
DistanceMT<TScalar,TPoint>::PointPair::PointPair() {
}

template <class TScalar, class TPoint>
inline bool DistanceMT<TScalar,TPoint>::PointPair::operator ==(const PointPair& other) const {
	return (first == other.first && second == other.second || first == other.second && second == other.first);
}

template <class TScalar, class TPoint>
inline bool DistanceMT<TScalar,TPoint>::PointPair::operator !=(const PointPair& other) const {
	return !operator ==(other);
}

} // Data
} // Math

#endif /* DISTANCEMT_H_ */
