#ifndef MESHTRANSFORM_H
#define MESHTRANSFORM_H

#include <utility>

#include "BoundingBox.h"
#include "MeshAnalysis.h"

namespace Geometry {

//
//	Note: All transformations return the parameters necessary to invert them
//	      (check their implementation for order of inverse transformation)
//

template <class MeshTraits>
class MeshTransform {
	public:
		typedef typename MeshTraits::MeshType    Mesh;
		typedef typename MeshTraits::ScalarType  Scalar;
		typedef typename MeshTraits::PointType   Point;
		typedef std::pair<Point, Scalar>         Transform;

	public:
		static Scalar scale(Mesh& mesh, Scalar factor);
		static Point  translate(Mesh& mesh, Point dir);

		static Transform fitToSphere(Mesh& mesh, Point center, Scalar radius);
		static Transform fitToUnitSphere(Mesh& mesh);
		static Transform fitToBox(Mesh& mesh, const BoundingBox<MeshTraits>& box);
		static Transform fitToBox(Mesh& mesh, const Point& min, const Point& max);
		static Transform fitTo01Cube(Mesh& mesh);
		static Transform fitToUnitCube(Mesh& mesh);
};


#include "MeshTransform.inl"

} /* Geometry */

#endif // MESHTRANSFORM_H
