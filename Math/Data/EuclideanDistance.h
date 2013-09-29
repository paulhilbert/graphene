#ifndef EUCLIDEANDISTANCE_H_
#define EUCLIDEANDISTANCE_H_

#include "../Vector.h"
#include "../Distances.h"

#include "Distance.h"

namespace Math {
namespace Data {

template <class Scalar, int Dim>
class EuclideanDistance : public Distance< Scalar, Vector<Scalar, Dim> > {
	public:
		typedef Vector<Scalar, Dim> Point;
		typedef typename Distance<Scalar, Point>::PointSet PointSet;

	public:
		EuclideanDistance(const PointSet& points) : Distance<Scalar, Point>(points) {}
		virtual ~EuclideanDistance() {}

	protected:
		Scalar computeDistance(Point x, Point y) { return ::Math::Distances::euclidean<Scalar, Dim>(x, y); }
};


} // Data
} // Math

#endif /* EUCLIDEANDISTANCE_H_ */
