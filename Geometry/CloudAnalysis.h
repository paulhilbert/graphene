#ifndef CLOUDANALYSIS_H
#define CLOUDANALYSIS_H

namespace Geometry {


template <class Points>
class CloudAnalysis {
	public:
		typedef typename Points::value_type Point;

	public:
		static Point centerOfMass(const Points& points, unsigned int normalizeAfter = 0);
};


#include "CloudAnalysis.inl"

} /* Geometry */

#endif // CLOUDANALYSIS_H
