#ifndef VECTOR_H
#define VECTOR_H


#include <cassert>
#include <iostream>
#include <cmath>


namespace Math {


template <typename Scalar, int DIM>
class Vector {
	public:
		typedef Scalar ScalarType;
		static inline int dim() { return DIM; }
		static inline int size() { return dim(); }

	public:
		Vector();
		Vector(const Scalar& v0, const Scalar& v1);
		Vector(const Scalar& v0, const Scalar& v1, const Scalar& v2);
		Vector(const Scalar& v0, const Scalar& v1, const Scalar& v2, const Scalar& v3);
		explicit Vector(const Scalar& v);
		explicit Vector(const Scalar values[DIM]);

		// do NOT remove non-template copy-constructor as it is important for the compiler
		Vector(const Vector<Scalar,DIM>& other);
		template <typename  OtherScalarType>
		Vector(const Vector<OtherScalarType,DIM>& other);

		// do NOT remove non-template assign-operator as it is important for the compiler
		Vector<Scalar,DIM>& operator=(const Vector<Scalar,DIM>& other);
		template <typename OtherScalarType>
		Vector<Scalar,DIM>& operator=(const Vector<OtherScalarType,DIM>& other);

		operator Scalar*();
		operator const Scalar*() const;


		Scalar& operator[](int i);
		const Scalar& operator[](int i) const;

		bool operator==(const Vector<Scalar,DIM>& other) const;
		bool operator!=(const Vector<Scalar,DIM>& other) const;

		Vector<Scalar,DIM> operator-(void) const;

		Vector<Scalar,DIM>& operator*=(const Scalar& s);
		Vector<Scalar,DIM>& operator/=(const Scalar& s);
		Vector<Scalar,DIM> operator*(const Scalar& s) const;
		Vector<Scalar,DIM> operator/(const Scalar& s) const;

		Vector<Scalar,DIM>& operator*=(const Vector<Scalar,DIM>& other);
		Vector<Scalar,DIM>& operator/=(const Vector<Scalar,DIM>& other);
		Vector<Scalar,DIM>& operator-=(const Vector<Scalar,DIM>& other);
		Vector<Scalar,DIM>& operator+=(const Vector<Scalar,DIM>& other);
		Vector<Scalar,DIM> operator*(const Vector<Scalar,DIM>& _v) const;
		Vector<Scalar,DIM> operator/(const Vector<Scalar,DIM>& _v) const;
		Vector<Scalar,DIM> operator+(const Vector<Scalar,DIM>& _v) const;
		Vector<Scalar,DIM> operator-(const Vector<Scalar,DIM>& _v) const;
		Vector<Scalar,3> operator%(const Vector<Scalar,3>& other) const;
		Vector<Scalar,3> operator%(const Vector<Scalar,2>& other) const;
		Scalar operator|(const Vector<Scalar,DIM>& other) const;

		Scalar norm() const;
		Scalar sum_sqr() const;

		Vector<Scalar,DIM>& normalize();

		Scalar max() const;
		Scalar min() const;
		int argMax() const;
		int argMin() const;
		Scalar mean() const;

		Vector<Scalar,DIM>& abs();


		template <class OtherScalarType, int OtherDim> friend class Vector;

	protected:
		Scalar m_values[DIM];
};


/* GLOBAL OPERATORS */

template<typename Scalar, int DIM>
inline Vector<Scalar,DIM> operator*(Scalar s, const Vector<Scalar,DIM>& v);

template<typename Scalar, int DIM>
inline Vector<Scalar,DIM> Normalize(const Vector<Scalar,DIM>& v);

template<typename Scalar, int DIM>
inline Vector<Scalar,DIM> Abs(const Vector<Scalar,DIM>& v);

template <typename Scalar,int DIM>
inline std::istream& operator>>(std::istream& is, Vector<Scalar,DIM>& vec);

template <typename Scalar,int DIM>
inline std::ostream& operator<<(std::ostream& os, const Vector<Scalar,DIM>& vec);


#include "Vector.inl"

#undef FOR_LOOP


} // Math


#endif 
