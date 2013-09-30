#ifndef TRIGONOMETRY_H_
#define TRIGONOMETRY_H_

#include <Math/Vector.h>

namespace Math {


template <class Type>
inline Vector<Type,3> polar2Cartesian(Vector<Type,2> p /*(theta, phi*/) {
	return Vec3(
		cos(p[1])*sin(p[0]),
		sin(p[1])*sin(p[0]),
		cos(p[0])
	);
}

template <class Type>
inline Vector<Type,3> polar2Cartesian(Vector<Type,3> p /*(theta, phi, r)*/) {
	return p[2] * Vec3(
		cos(p[1])*sin(p[0]),
		sin(p[1])*sin(p[0]),
		cos(p[0])
	);
}

template <class Type>
inline Type cartesian2PolarPhi(Vector<Type,2> p) {
	float phi = atan2(p[1], p[0]);
	if (phi < 0) phi = 2*M_PI + phi;
	return phi;
}

template <class Type>
inline Type cartesian2PolarPhi(Vector<Type,3> p) {
	float phi = atan2(p[1], p[0]);
	if (phi < 0) phi = 2*M_PI + phi;
	return phi;
}

template <class Type>
inline Type cartesian2PolarTheta(Vector<Type,3> p) {
	return acos(p[2] / p.norm());
}

template <class Type>
inline Vector<Type,2> cartesian2Polar(Vector<Type,3> p) {
	return Vector<Type,2>( cartesian2PolarTheta, cartesian2PolarPhi );
}

template <class Type>
inline Vector<Type,3> cartesian2Polar(Vector<Type,3> p) {
	return Vector<Type,3>( cartesian2PolarTheta, cartesian2PolarPhi, p.norm() );
}

template<typename T>
float gaussianTrapezEquation(const T &pointList, const int &size) {
	if (size < 3)
		return 0.f;
	float area = 0.f;
	for (int i=1; i < size; ++i)
		area += ( (pointList[i][0] + pointList[i-1][0])
			  * (pointList[i-1][1] - pointList[i][1]) );

	area += ( (pointList[0][0] + pointList[size-1][0])
		  * (pointList[size-1][1] - pointList[0][1]) );

	return .5f * fabs(area);
}

template<typename T>
float gaussianTrapezEquation(const T &pointList) {
	return gaussianTrapezEquation(pointList, (int)pointList.size());
}

} // Math

#endif /* TRIGONOMETRY_H_ */
