template <class MeshTraits>
BoundingBox<MeshTraits>::BoundingBox() : m_min(0,0,0), m_max(0,0,0), m_range(0,0,0) {
}

template <class MeshTraits>
BoundingBox<MeshTraits>::BoundingBox(const Point& min, const Point& max, const Point& margin) {
	compute(min, max, margin);
}

template <class MeshTraits>
BoundingBox<MeshTraits>::BoundingBox(const Mesh& mesh, const Point& margin) {
	compute(mesh, margin);
}

template <class MeshTraits>
template <class PointSet>
BoundingBox<MeshTraits>::BoundingBox(const PointSet& points, const Point& margin) {
	compute(points, margin);
}

template <class MeshTraits>
BoundingBox<MeshTraits>::~BoundingBox() {
}

template <class MeshTraits>
inline void BoundingBox<MeshTraits>::compute(const Point& min, const Point& max, const Point& margin) {
	m_min = min - margin;
	m_max = max + margin;
	m_range = max - min;
}

template <class MeshTraits>
inline void BoundingBox<MeshTraits>::compute(const Mesh& mesh, const Point& margin) {
	auto points = MeshTraits::pointSet(mesh);
	compute(points, margin);
}

template <class MeshTraits>
template <class PointSet>
void BoundingBox<MeshTraits>::compute(const PointSet& points, const Point& margin) {
	auto comp = [](int dim, Point a, Point b) { return a[dim] < b[dim]; };
	auto minX = std::min_element(points.begin(), points.end(), std::bind(comp, 0, std::placeholders::_1, std::placeholders::_2));
	auto maxX = std::max_element(points.begin(), points.end(), std::bind(comp, 0, std::placeholders::_1, std::placeholders::_2));
	auto minY = std::min_element(points.begin(), points.end(), std::bind(comp, 1, std::placeholders::_1, std::placeholders::_2));
	auto maxY = std::max_element(points.begin(), points.end(), std::bind(comp, 1, std::placeholders::_1, std::placeholders::_2));
	auto minZ = std::min_element(points.begin(), points.end(), std::bind(comp, 2, std::placeholders::_1, std::placeholders::_2));
	auto maxZ = std::max_element(points.begin(), points.end(), std::bind(comp, 2, std::placeholders::_1, std::placeholders::_2));
	m_min = Point( (*minX)[0], (*minY)[1], (*minZ)[2] );
	m_max = Point( (*maxX)[0], (*maxY)[1], (*maxZ)[2] );
	m_range = m_max - m_min;
}

template <class MeshTraits>
inline void BoundingBox<MeshTraits>::addPoint(const Point& point) {
	if (m_min[0] > point[0]) m_min[0] = point[0];
	if (m_min[1] > point[1]) m_min[1] = point[1];
	if (m_min[2] > point[2]) m_min[2] = point[2];
	if (m_max[0] < point[0]) m_max[0] = point[0];
	if (m_max[1] < point[1]) m_max[1] = point[1];
	if (m_max[2] < point[2]) m_max[2] = point[2];
	m_range = m_max - m_min;
}

template <class MeshTraits>
inline typename BoundingBox<MeshTraits>::Point BoundingBox<MeshTraits>::getMin() const {
	return m_min;
}

template <class MeshTraits>
inline typename BoundingBox<MeshTraits>::Point BoundingBox<MeshTraits>::getMax() const {
	return m_max;
}

template <class MeshTraits>
inline typename BoundingBox<MeshTraits>::Point BoundingBox<MeshTraits>::getRange() const {
	return m_range;
}

template <class MeshTraits>
inline typename BoundingBox<MeshTraits>::Point BoundingBox<MeshTraits>::getCenter() const {
	return m_min + (Scalar(0.5) * m_range);
}

template <class MeshTraits>
inline unsigned int BoundingBox<MeshTraits>::getMinDim() const {
	// this is an argmin
	return std::min({0,1,2}, [=](int a, int b) { return m_range[a] < m_range[b]; });
}

template <class MeshTraits>
inline unsigned int BoundingBox<MeshTraits>::getMaxDim() const {
	// this is an argmax
	return std::max({0,1,2}, [=](unsigned int a, unsigned int b) { return m_range[a] < m_range[b]; });
}

template <class MeshTraits>
inline typename BoundingBox<MeshTraits>::Scalar BoundingBox<MeshTraits>::getMinRange() const {
	return std::min({m_range[0], m_range[1], m_range[2]});
}

template <class MeshTraits>
inline typename BoundingBox<MeshTraits>::Scalar BoundingBox<MeshTraits>::getMaxRange() const {
	return std::max({m_range[0], m_range[1], m_range[2]});
}

template <class MeshTraits>
inline void BoundingBox<MeshTraits>::setMin(const Point& min) {
	m_min = min;
}

template <class MeshTraits>
inline void BoundingBox<MeshTraits>::setMax(const Point& max) {
	m_max = max;
}

template <class MeshTraits>
inline void BoundingBox<MeshTraits>::setRange(const Point& range) {
	m_range = range;
}

template <class MeshTraits>
inline void BoundingBox<MeshTraits>::translate(const Point& dir) {
	m_min += dir;
	m_max += dir;
}

template <class MeshTraits>
inline void BoundingBox<MeshTraits>::scale(Scalar factor) {
	m_min *= factor;
	m_max *= factor;
	m_range = m_max - m_min;
}
