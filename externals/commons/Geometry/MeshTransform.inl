template <class MeshTraits>
typename MeshTransform<MeshTraits>::Scalar MeshTransform<MeshTraits>::scale(Mesh& mesh, Scalar factor) {
	MeshTraits::mapPoints(mesh, [=](Point& p) { p = factor * p; });
	return Scalar(1) / factor;
}

template <class MeshTraits>
typename MeshTransform<MeshTraits>::Point MeshTransform<MeshTraits>::translate(Mesh& mesh, Point dir) {
	MeshTraits::mapPoints(mesh, [=](Point& p) { p = p + dir; });
	return -dir;
}

template <class MeshTraits>
typename MeshTransform<MeshTraits>::Transform MeshTransform<MeshTraits>::fitToSphere(Mesh& mesh, Point center, Scalar radius) {
	Point centroid = MeshAnalysis<MeshTraits>::vertexCOM(mesh);
	Scalar crtRadius = Scalar(0);
	MeshTraits::mapPoints(mesh, [&](Point& p) { crtRadius = std::max(MeshTraits::pointNorm(p-centroid), crtRadius); });
	Scalar factor = radius / crtRadius;
	Scalar invScale = scale(mesh, factor);
	Point  invTrans = translate(mesh, center - factor*centroid);
	return std::make_pair(invTrans, invScale);
}

template <class MeshTraits>
typename MeshTransform<MeshTraits>::Transform MeshTransform<MeshTraits>::fitToUnitSphere(Mesh& mesh) {
	return fitToSphere(mesh, Point(0,0,0), Scalar(1));
}

template <class MeshTraits>
typename MeshTransform<MeshTraits>::Transform MeshTransform<MeshTraits>::fitToBox(Mesh& mesh, const BoundingBox<MeshTraits>& box) {
	return fitToBox(mesh, box.getMin(), box.getMax());
}

template <class MeshTraits>
typename MeshTransform<MeshTraits>::Transform MeshTransform<MeshTraits>::fitToBox(Mesh& mesh, const Point& min, const Point& max) {
	BoundingBox<MeshTraits> bb(mesh);
	Point range = max-min;
	unsigned int maxDim = bb.getMaxDim();
	Scalar factor = range[maxDim] / bb.getRange()[maxDim];
	Scalar invScale = scale(mesh, factor);
	bb.scale(factor);
	Point  invTrans = translate(mesh, min - bb.getMin());
	return std::make_pair(invTrans, invScale);
}

template <class MeshTraits>
typename MeshTransform<MeshTraits>::Transform MeshTransform<MeshTraits>::fitTo01Cube(Mesh& mesh) {
	return fitToBox(mesh, Point(0,0,0), Point(1,1,1));
}

template <class MeshTraits>
typename MeshTransform<MeshTraits>::Transform MeshTransform<MeshTraits>::fitToUnitCube(Mesh& mesh) {
	return fitToBox(mesh, Point(-1,-1,-1), Point(1,1,1));
}
