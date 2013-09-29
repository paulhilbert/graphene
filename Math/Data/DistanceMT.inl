template <class Scalar, class Point>
DistanceMT<Scalar,Point>::DistanceMT(const PointSet& points) : Distance<Scalar,Point>(points) {
}

template <class Scalar, class Point>
DistanceMT<Scalar,Point>::~DistanceMT() {
}

template <class Scalar, class Point>
Scalar DistanceMT<Scalar,Point>::compute(PointPair p) {
	return computeDistanceMT(m_points[p.first], m_points[p.second]);
}
