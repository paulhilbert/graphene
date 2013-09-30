template <class T>
inline Matrix<T,3,1> up(const Matrix<T,3,1>& ref) {
	float d = ref[2] / ref.norm();
	if (1.f-std::abs(d) < 0.000001f) return Matrix<T,3,1>::UnitX();
	return Matrix<T,3,1>::UnitZ();
}

template <class T>
inline Matrix<T,3,1> right(const Matrix<T,3,1>& dir) {
	Matrix<T,3,1> right(dir);
	right.normalize();
	right = right.cross(up(right));
	right.normalize();
	return right;
}

template <class Scalar, int Rows, int Cols>
inline void clamp(Matrix<Scalar, Rows, Cols>& m, Scalar min, Scalar max) {
	m.unaryExpr([&](Scalar value) { return value > max ? max : (value < min ? min : value); });
}
