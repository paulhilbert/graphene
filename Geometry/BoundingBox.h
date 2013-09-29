#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H


namespace Geometry {


template <class MeshTraits>
class BoundingBox {
	public:
		typedef typename MeshTraits::MeshType    Mesh;
		typedef typename MeshTraits::ScalarType  Scalar;
		typedef typename MeshTraits::PointType   Point;
		
	public:
		BoundingBox();
		BoundingBox(const Point& min, const Point& max, const Point& margin = Point(0,0,0));
		BoundingBox(const Mesh& mesh, const Point& margin = Point(0,0,0));
		template <class PointSet>
		BoundingBox(const PointSet& points, const Point& margin = Point(0,0,0));
		virtual ~BoundingBox();

		void compute(const Point& min, const Point& max, const Point& margin = Point(0,0,0));
		void compute(const Mesh& mesh, const Point& margin = Point(0,0,0));
		template <class PointSet>
		void compute(const PointSet& points, const Point& margin = Point(0,0,0));

		void addPoint(const Point& point);

		Point getMin() const;
		Point getMax() const;
		Point getRange() const;
		Point getCenter() const;
		unsigned int getMinDim() const;
		unsigned int getMaxDim() const;
		Scalar getMinRange() const;
		Scalar getMaxRange() const;
		void setMin(const Point& min);
		void setMax(const Point& max);
		void setRange(const Point& range);

		void translate(const Point& dir);
		void scale(Scalar factor);
	
	protected:
		Point m_min;
		Point m_max;
		Point m_range;
};


#include "BoundingBox.inl"


} /* Geometry */

#endif // BOUNDINGBOX_H
