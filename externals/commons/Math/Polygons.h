#ifndef POLYGONS_H
#define POLYGONS_H

#include <boost/shared_ptr.hpp>

#include "Vector.h"
#include "Trigonometry.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/create_straight_skeleton_2.h>

namespace Math {

template <class Point>
class Polygons {
	public:
		typedef vector<Point> Polygon;
		typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
		typedef K::Point_2 CGALPoint;
		typedef CGAL::Polygon_2<K> CGALPoly;
		typedef CGAL::Straight_skeleton_2<K> Ss;
		typedef boost::shared_ptr<Ss> SsPtr;

		static float relativeSkeletonLength(const Polygon& p) {
			float area = gaussianTrapezEquation(p);
			CGALPoly poly;
			for (unsigned int i=0; i<p.size(); ++i) poly.push_back( CGALPoint(p[i][0], p[i][1]) );
			if (!poly.size() || !poly.is_simple()) return 0.f;
			SsPtr iss = CGAL::create_interior_straight_skeleton_2(poly.vertices_begin(), poly.vertices_end());
			set<int> visited;
			float sum = 0.f;
			for (Ss::Halfedge_iterator it = iss->halfedges_begin(); it != iss->halfedges_end(); ++it) {
				if (!it->is_inner_bisector()) continue;
				if (visited.find(it->id()) != visited.end()) continue;
				Ss::Vertex target = *(it->vertex());
				CGALPoint pTarget = target.point();
				Ss::Vertex source = *(it->prev()->vertex());
				CGALPoint pSource = source.point();
				visited.insert(it->id());
				visited.insert(it->opposite()->id());
				float time = 0.5f*(source.time() + target.time());
				float dist = sqrt( (pTarget.x() - pSource.x())*(pTarget.x() - pSource.x()) + (pTarget.y() - pSource.y())*(pTarget.y() - pSource.y()) );
				sum += dist / time;
			}

			if (area) sum /= sqrt(area);
			return sum;
		}

		static SsPtr straightSkeleton(const Polygon& p) {
			CGALPoly poly;
			for (unsigned int i=0; i<p.size(); ++i) poly.push_back( CGALPoint(p[i][0], p[i][1]) );
			return CGAL::create_interior_straight_skeleton_2(poly.vertices_begin(), poly.vertices_end());
		}
};

} // Math

#endif
