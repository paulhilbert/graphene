inline Matrix4f scale(const Matrix4f& m, const Vector3f& scale) {
	Matrix4f result(m);
	for (int i=0; i<3; ++i) result.col(i) *= scale[i];
	return result;
}

inline Matrix4f translate(const Matrix4f& m, const Vector3f& translate) {
	Matrix4f t = Matrix4f::Identity();
	t.block<3,1>(0,3) = translate;
	return m*t;
	/*
	Matrix4f result(m);
	for (int i=0; i<3; ++i) result.col(3) += result.col(i) * translate[i];
	return result;
	*/
}

inline Matrix4f ortho(float left, float right, float bottom, float top, float zNear, float zFar) {
	Matrix4f result = Matrix4f::Identity();
	result(0,0) = 2.f / (right - left);
	result(1,1) = 2.f / (top - bottom);
	result(2,2) = - 2.f / (zFar - zNear);
	result(0,3) = - (right + left) / (right - left);
	result(1,3) = - (top + bottom) / (top - bottom);
	result(2,3) = - (zFar + zNear) / (zFar - zNear);
	return result;
}

inline Matrix4f ortho(float left, float right, float bottom, float top) {
	Matrix4f result = Matrix4f::Identity();
	result(0,0) = 2.f / (right - left);
	result(1,1) = 2.f / (top - bottom);
	result(2,2) = - 1.f;
	result(0,3) = - (right + left) / (right - left);
	result(1,3) = - (top + bottom) / (top - bottom);
	return result;
}

inline Matrix4f perspective( float fovy, float aspect,	float zNear, float zFar) {
	float range = static_cast<float>(std::tan((fovy / 2.f) * M_PI / 180.f) * zNear);	
	float left = -range * aspect;
	float right = range * aspect;
	float bottom = -range;
	float top = range;

	Matrix4f result = Matrix4f::Zero();
	result(0,0) = (2.f * zNear) / (right - left);
	result(1,1) = (2.f * zNear) / (top - bottom);
	result(2,2) = - (zFar + zNear) / (zFar - zNear);
	result(3,2) = - 1.f;
	result(2,3) = - (2.f * zFar * zNear) / (zFar - zNear);
	return result;
}

inline Matrix4f frustum( float left, float right, float bottom, float top, float nearVal, float farVal ) {
	Matrix4f result = Matrix4f::Zero();
	result(0,0) = (2.f * nearVal) / (right - left);
	result(1,1) = (2.f * nearVal) / (top - bottom);
	result(2,0) = (right + left) / (right - left);
	result(2,1) = (top + bottom) / (top - bottom);
	result(2,2) = -(farVal + nearVal) / (farVal - nearVal);
	result(3,2) = -1.f;
	result(2,3) = -(2.f * farVal * nearVal) / (farVal - nearVal);
	return result;
}

template <class U>
inline Vector3f project(const Vector3f& obj, const Matrix4f& model, const Matrix4f& proj, const Matrix<U,4,1>& viewport) {
	Vector4f tmp(obj[0], obj[1], obj[2], 1.f);
	tmp = model * tmp;
	tmp = proj * tmp;

	tmp /= tmp[3];
	tmp = tmp * 0.5f + Vector4f(0.5f,0.5f,0.5f,0.5f);
	tmp[0] = tmp[0] * static_cast<float>(viewport[2]) + static_cast<float>(viewport[0]);
	tmp[1] = tmp[1] * static_cast<float>(viewport[3]) + static_cast<float>(viewport[1]);

	return Vector3f(tmp[0], tmp[1], tmp[2]);
}

template <class U>
inline Vector3f unProject(const Vector3f& win, const Matrix4f& model, const Matrix4f& proj, const Matrix<U,4,1>& viewport) {
	Matrix4f inverse = (proj * model).inverse();

	Vector4f tmp(win[0], win[1], win[2], 1.f);
	tmp[0] = (tmp[0] - static_cast<float>(viewport[0])) / static_cast<float>(viewport[2]);
	tmp[1] = (tmp[1] - static_cast<float>(viewport[1])) / static_cast<float>(viewport[3]);
	tmp = tmp * 2.f - Vector4f(1.f, 1.f, 1.f, 1.f);

	Vector4f obj = inverse * tmp;
	obj /= obj[3];

	return Vector3f(obj[0], obj[1], obj[2]);
}
