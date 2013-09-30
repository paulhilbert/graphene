#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <cmath>

#include <vector>
using std::vector;

#include <tr1/memory>
using std::tr1::shared_ptr;

#include "Vector.h"

namespace Math {

template <class T>
class Quaternion {
	protected:
		typedef Vector<T,3> BVec;

	public:
		Quaternion() : m_scalar(T(0)), m_bivector(T(0)) {}
		explicit Quaternion(T v) : m_scalar(v), m_bivector(T(v)) {}
		Quaternion(T s, T b0, T b1, T b2) : m_scalar(s), m_bivector(b0, b1, b2) {}
		Quaternion(BVec b) : m_scalar(T(0)), m_bivector(b) {} 
		Quaternion(T s, BVec b) : m_scalar(s), m_bivector(b) {}

		static Quaternion<T>* Rotation(T angle, BVec axis) {
			Quaternion<T>* rot = new Quaternion<T>();
			rot->makeRotation(angle, axis);

			return rot;
		}

		virtual ~Quaternion() {}

		T getScalar();
		BVec getBivector();
		// returns scalar (read-write)
		T& operator ()();
		// returns scalar (read-only)
		const T& operator ()() const;
		// returns i'th bivector coefficient (read/write)
		T& operator [](int i);
		// returns i'th bivector coefficient (read-only)
		const T& operator [](int i) const;
		
		// negation
		Quaternion<T> operator -() const;
		// conjugation
		Quaternion<T> operator !() const;
		// inverse
		Quaternion<T> operator ~() const;

		Quaternion<T>& operator *=(const T& scalar);
		Quaternion<T>  operator * (const T& scalar) const;
		Quaternion<T>& operator /=(const T& scalar);
		Quaternion<T>  operator / (const T& scalar) const;

		Quaternion<T>& operator +=(const Quaternion<T>& other);
		Quaternion<T>  operator + (const Quaternion<T>& other) const;
		Quaternion<T>& operator -=(const Quaternion<T>& other);
		Quaternion<T>  operator - (const Quaternion<T>& other) const;
		Quaternion<T>& operator *=(const Quaternion<T>& other);
		Quaternion<T>  operator * (const Quaternion<T>& other) const;
		Quaternion<T>& operator /=(const Quaternion<T>& other);
		Quaternion<T>  operator / (const Quaternion<T>& other) const;

		T sum_sqr() const;
		T norm() const;
		Quaternion<T> getUnit();
		Quaternion<T> makeUnit() const;

		void makeRotation(T angle, BVec axis);
		BVec rotate(BVec original);

	protected:
		T            m_scalar;
		Vector<T,3>  m_bivector;
};


template <class T>
inline T Quaternion<T>::getScalar() {
	return m_scalar;
}
template <class T>
inline Vector<T,3> Quaternion<T>::getBivector() {
	return m_bivector;
}
// returns scalar (read-write)
template <class T>
T& Quaternion<T>::operator ()() {
	return m_scalar;
}
// returns scalar (read-only)
template <class T>
const T& Quaternion<T>::operator ()() const {
	return m_scalar;
}
// returns i'th bivector coefficient (read/write)
template <class T>
T& Quaternion<T>::operator [](int i) {
	assert(i < m_bivector.dim());
	return m_bivector[i];
}
// returns i'th bivector coefficient (read-only)
template <class T>
const T& Quaternion<T>::operator [](int i) const {
	assert(i < m_bivector.dim());
	return m_bivector[i];
}


// (component-wise) negation
template <class T>
Quaternion<T> Quaternion<T>::operator -() const {
	return Quaternion<T>( -m_scalar, -m_bivector );
}
// conjugation
template <class T>
Quaternion<T> Quaternion<T>::operator !() const {
	return Quaternion<T>( m_scalar, -m_bivector );
}
// inverse
template <class T>
Quaternion<T> Quaternion<T>::operator ~() const {
	return ((!(*this)) / this->sum_sqr());
}


template <class T>
Quaternion<T>& Quaternion<T>::operator *=(const T& scalar) {
	m_scalar *= scalar;
	m_bivector *= scalar;
	return *this;
}
template <class T>
Quaternion<T>  Quaternion<T>::operator * (const T& scalar) const {
	Quaternion<T> copy(*this);
	copy *= scalar;
	return copy;
}
template <class T>
Quaternion<T>& Quaternion<T>::operator /=(const T& scalar) {
	T factor = T(1) / scalar;
	return ((*this) *= factor);
}
template <class T>
Quaternion<T>  Quaternion<T>::operator / (const T& scalar) const {
	Quaternion<T> copy(*this);
	copy /= scalar;
	return copy;
}


template <class T>
Quaternion<T>& Quaternion<T>::operator +=(const Quaternion<T>& other) {
	m_scalar += other.m_scalar;
	m_bivector += other.m_bivector;
	return *this;
}
template <class T>
Quaternion<T>  Quaternion<T>::operator + (const Quaternion<T>& other) const {
	Quaternion<T> copy(*this);
	copy += other;
	return copy;
}
template <class T>
Quaternion<T>& Quaternion<T>::operator -=(const Quaternion<T>& other) {
	m_scalar -= other.m_scalar;
	m_bivector -= other.m_bivector;
	return *this;
}
template <class T>
Quaternion<T>  Quaternion<T>::operator - (const Quaternion<T>& other) const {
	Quaternion<T> copy(*this);
	copy -= other;
	return copy;
}
template <class T>
Quaternion<T>& Quaternion<T>::operator *=(const Quaternion<T>& other) {
	double s[9], t;
	s[0] = ((*this)[2]-(*this)[1]) * (other[1]-other[2]);
	s[1] = ((*this)()+(*this)[0]) * (other()+other[0]);
	s[2] = ((*this)()-(*this)[0]) * (other[1]+other[2]);
	s[3] = ((*this)[2]+(*this)[1]) * (other()-other[0]);
	s[4] = ((*this)[2]-(*this)[0]) * (other[0]-other[1]);
	s[5] = ((*this)[2]+(*this)[0]) * (other[0]+other[1]);
	s[6] = ((*this)()+(*this)[1]) * (other()-other[2]);
	s[7] = ((*this)()-(*this)[1]) * (other()+other[2]);
	s[8] = s[5]+s[6]+s[7];
	t = (s[4]+s[8])*0.5;

	m_scalar = s[0]+t-s[5];
	m_bivector[0] = s[1]+t-s[8];
	m_bivector[1] = s[2]+t-s[7];
	m_bivector[2] = s[3]+t-s[6];

	return *this;
}
template <class T>
Quaternion<T>  Quaternion<T>::operator * (const Quaternion<T>& other) const {
	Quaternion<T> copy(*this);
	copy *= other;
	return copy;
}
template <class T>
Quaternion<T>& Quaternion<T>::operator /=(const Quaternion<T>& other) {
	return ((*this) *= (~other));
}
template <class T>
Quaternion<T>  Quaternion<T>::operator / (const Quaternion<T>& other) const {
	Quaternion<T> copy(*this);
	copy -= other;
	return copy;
}


template <class T>
inline T Quaternion<T>::sum_sqr() const {
	return (m_scalar * m_scalar) + m_bivector.sum_sqr();
}
template <class T>
inline T Quaternion<T>::norm() const {
	return sqrt(sum_sqr());
}
template <class T>
inline Quaternion<T> Quaternion<T>::getUnit() {
	Quaternion<T> copy(*this);
	return copy.makeUnit();
}
template <class T>
inline Quaternion<T> Quaternion<T>::makeUnit() const {
	*this /= norm();
	return *this;
}


template <class T>
inline void Quaternion<T>::makeRotation(T angle, Quaternion<T>::BVec axis) {
	Vector<T,3> rotaxis = axis;
	if (axis.sum_sqr() - T(1) > T(0.0001)) rotaxis.normalize();
	T radangle = angle * (M_PI / 360.f);
	m_scalar = cos(radangle);
	m_bivector = rotaxis * sin(radangle);
}
template <class T>
inline Vector<T,3> Quaternion<T>::rotate(BVec original) {
	Quaternion<T> p(original);
	p = (*this) * (p * (!(*this)));
	return p.m_bivector;
}


template <class T>
Vector<T,3> rotateVector(Vector<T,3> original, T angle, Vector<T,3> axis) {
	shared_ptr< Quaternion<T> > rot(Quaternion<T>::Rotation(angle, axis));
	return rot->rotate(original);
}


} // Math

#endif /* QUATERNION_H_ */
