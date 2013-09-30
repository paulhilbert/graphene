inline Ray::Ray(Vec direction, Vec origin) : m_direction(direction), m_origin(origin) {
	m_direction.normalize();
}

inline Ray::Vec Ray::getOrigin() const {
	return m_origin;
}

inline Ray::Vec Ray::getDirection() const {
	return m_direction;
}

inline void Ray::setOrigin(Vec origin) {
	m_origin = origin;
}

inline void Ray::setDirection(Vec direction) {
	m_direction = direction;
}

inline Ray::Vec Ray::eval(float t) const {
	return m_origin + m_direction * t;
}

inline void Ray::setPoints(Vec p0, Vec p1) {
	m_origin = p0;
	m_direction = p1 - p0;
	m_direction.normalize();
}

inline Ray::OptVec Ray::intersectRay(const Ray& other) const {
	float d11 = m_direction[0];
	float d12 = m_direction[1];
	float d21 = -other.m_direction[0];
	float d22 = -other.m_direction[1];

	float det = d11 * d22 - d12 * d21;

	float o11 = m_origin[0];
	float o12 = m_origin[1];
	float o21 = other.m_origin[0];
	float o22 = other.m_origin[1];

	float t0, t1;

	t0 = d22 * (o21-o11) - d21 * (o22-o12);
	t0 /= det;
	if ( (t0 > 0.f) && (t0 < 1.f) ) {
		t1 = -d12 * (o21-o11) + d11 * (o22-o12);
		t1 /= det;
		if ( (t1 > 0.f) && (t1 < 1.f) ) {
			OptVec result;
			result = m_origin + t0 * m_direction;
			return result;
		} else {
			return boost::none;
		}
	}

	return boost::none;
}

inline bool Ray::intersectSphere(const Vec& c, float r, Vec& intersection, float& lambda, Limits limits) const {
	Vec oc = c - m_origin;
	float l2oc = oc.squaredNorm();
	if (l2oc < r*r) { // starts inside of the sphere
		float tca = oc.dot(m_direction);			// omit division if ray.d is normalized
		float l2hc = (r*r - l2oc) + tca*tca;  // division
		lambda = tca + sqrt(l2hc);
		intersection = eval(lambda);
		return lambda >> limits;
	} else {
		float tca = oc.dot(m_direction);
		if (tca < 0) // points away from the sphere
			return false;
		float l2hc = (r*r - l2oc) + (tca*tca);	// division
		if (l2hc > 0) {
			lambda = tca - sqrt(l2hc);
			intersection = eval(lambda);
			return lambda >> limits;
		}
		return false;
	}
}

inline bool Ray::intersectTriangle(const Triangle& t, Vec& intersection, float& lambda, Limits limits) const {
	Vec e1, e2;
	e1 = std::get<1>(t)-std::get<0>(t);
	e2 = std::get<2>(t)-std::get<0>(t);

	Eigen::Vector3f h = e1.cross(e2);
	float a = e1.dot(h);

	if (a > -0.00001 && a < 0.00001)	return false;

	float f = 1.f / a;
	Vec s = m_origin - std::get<0>(t);
	float u = f * s.dot(h);

	if (u < 0.0 || u > 1.0)	return false;

	h = s.cross(e1);
	float v = f * s.dot(h);

	if (v < 0.0 || u + v > 1.0) return false;

	lambda = f * e2.dot(h);
	intersection = eval(lambda);

	return lambda >> limits;
}

inline float Ray::distance(const Ray& other, Limits limitThis, Limits limitOther, Vec* pointThis, Vec* pointOther) const {
	if ((1.f - fabs(m_direction.dot(other.m_direction))) < std::numeric_limits<float>::epsilon()) {
		Vec temp = eval((other.m_origin - m_origin).dot(m_direction));
		return (temp - other.m_origin).norm();
	}

	const Vec& P1 = m_origin;
	const Vec& U1 = m_direction;
	const Vec& P2 = other.m_origin;
	const Vec& U2 = other.m_direction;

	Vec M = U2.cross(U1);
	float m2 = M.squaredNorm();
	Vec R = (P2-P1).cross(M) / m2;

	float t0 = R.dot(U2);
	float t1 = R.dot(U1);

	// use end points if t0 or t1 exceed desired range
	if ( !(t0 >> limitThis) ) {
		t0 = t0 < limitThis.first ? limitThis.first : limitThis.second;
	}
	if ( !(t1 >> limitOther) ) {
		t1 = t1 < limitOther.first ? limitOther.first : limitOther.second;
	}

	Vec pThis = eval(t0);
	Vec pOther = other.eval(t1);
	if (pointThis) *pointThis = pThis;
	if (pointOther) *pointOther = pOther;

	return (pThis - pOther).norm();
}

