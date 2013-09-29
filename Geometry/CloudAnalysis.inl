template <class Points>
typename CloudAnalysis<Points>::Point CloudAnalysis<Points>::centerOfMass(const Points& points, unsigned int normalizeAfter) {
	// you should think about first fitting the point set into a unit
	// cube for numeric reasons
	std::vector<Point> subCentroids;
	subCentroids.push_back(Point(0,0,0));
	unsigned int i=0;
	for (auto it=points.begin(); it!=points.end(); ++it, ++i) {
		if ( (normalizeAfter > 0) && (i!=0) && (i%normalizeAfter == 0) ) {
			subCentroids.back() /= static_cast<float>(points.size());
			subCentroids.push_back(Point(0,0,0));
		}
		subCentroids.back() += *it;
	}
	subCentroids.back() /= static_cast<float>(points.size());
	if (normalizeAfter == 0) return subCentroids.back();
	Point centroid;
	for (auto it=subCentroids.begin(); it != subCentroids.end(); ++it) {
		centroid += *it;
	}
	return centroid;
}
