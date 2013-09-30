#ifndef MATRIXT_H
#define MATRIXT_H

#include <cassert>
#include <iostream>
#include <cmath>

#include <Math/Vector.h>

namespace Math {


template <typename Scalar, int ROWS, int COLS>
class Matrix {
	public:
		typedef Scalar ScalarType;

		static inline int rows() { return ROWS; }
		static inline int cols() { return COLS; }
		static inline int size() { return ROWS*COLS; }

	public:
		Matrix();
		Matrix(const Scalar& r00, const Scalar& r01, const Scalar& r10, const Scalar& r11);
		Matrix(const Scalar& r00, const Scalar& r01, const Scalar& r02, 
		       const Scalar& r10, const Scalar& r11, const Scalar& r12,
		       const Scalar& r20, const Scalar& r21, const Scalar& r22 );
		Matrix(const Scalar& r00, const Scalar& r01, const Scalar& r02, const Scalar& r03,
		       const Scalar& r10, const Scalar& r11, const Scalar& r12, const Scalar& r13,
		       const Scalar& r20, const Scalar& r21, const Scalar& r22, const Scalar& r23,
		       const Scalar& r30, const Scalar& r31, const Scalar& r32, const Scalar& r33 );
		explicit Matrix(const Scalar& v);
		explicit Matrix(const Scalar values[ROWS*COLS]);

		static Matrix<Scalar,ROWS,COLS>* unity();

		template<typename otherScalarType>
		explicit Matrix(const Matrix<otherScalarType,ROWS,COLS>& other);

		template<typename otherScalarType>
		Matrix<Scalar,ROWS,COLS>& operator=(const Matrix<otherScalarType,ROWS,COLS>& other);

		operator Scalar*();
		operator const Scalar*() const;

		Scalar& operator()(int i, int j);
		const Scalar& operator()(int i, int j) const;
		Scalar& operator()(size_t i, size_t j);
		const Scalar& operator()(size_t i, size_t j) const;

		Vector<Scalar,COLS> getRow(int i) const;
		Vector<Scalar,ROWS> getColumn(int j) const;
		void setRow(int i, Vector<Scalar,COLS> row);
		void setColumn(int j, Vector<Scalar,ROWS> col);

		bool operator==(const Matrix<Scalar,ROWS,COLS>& other) const;
		bool operator!=(const Matrix<Scalar,ROWS,COLS>& other) const;

		Matrix<Scalar,ROWS,COLS> operator-(void) const;

		Matrix<Scalar,ROWS,COLS>& operator*=(const Scalar& s);
		Matrix<Scalar,ROWS,COLS>& operator/=(const Scalar& s);
		Matrix<Scalar,ROWS,COLS> operator*(const Scalar& s) const;
		Matrix<Scalar,ROWS,COLS> operator/(const Scalar& s) const;

		Vector<Scalar,ROWS> operator*(const Vector<Scalar,COLS>& v) const;

		Matrix<Scalar,ROWS,COLS>& operator*=(const Matrix<Scalar,COLS,ROWS>& other);
		Matrix<Scalar,ROWS,COLS>& operator+=(const Matrix<Scalar,ROWS,COLS>& other);
		Matrix<Scalar,ROWS,COLS>& operator-=(const Matrix<Scalar,ROWS,COLS>& other);
		Matrix<Scalar,ROWS,COLS> operator*(const Matrix<Scalar,COLS,ROWS>& _v) const;
		Matrix<Scalar,ROWS, COLS> operator+(const Matrix<Scalar,ROWS,COLS>& _v) const;
		Matrix<Scalar,ROWS,COLS> operator-(const Matrix<Scalar,ROWS,COLS>& _v) const;

		void transpose();
		Matrix<Scalar,ROWS,COLS> getTranspose();

	public:
		Scalar m_values[ROWS*COLS];
};




/* IMPLEMENTATIONS */

template <typename Scalar, int ROWS, int COLS>
inline Matrix<Scalar,ROWS,COLS>::Matrix() {
}

template <typename Scalar, int ROWS, int COLS>
Matrix<Scalar,ROWS,COLS>::Matrix(const Scalar& r00, const Scalar& r01, const Scalar& r10, const Scalar& r11) {
	assert(ROWS==2 && COLS==2);
	m_values[0] = r00; m_values[1] = r01; m_values[2] = r10; m_values[3] = r11;
}

template <typename Scalar, int ROWS, int COLS>
Matrix<Scalar,ROWS,COLS>::Matrix(const Scalar& r00, const Scalar& r01, const Scalar& r02, 
                                 const Scalar& r10, const Scalar& r11, const Scalar& r12, 
                                 const Scalar& r20, const Scalar& r21, const Scalar& r22 ) {
	assert(ROWS==3 && COLS ==3);
	m_values[0]=r00; m_values[1]=r01; m_values[2]=r02;
	m_values[3]=r10; m_values[4]=r11; m_values[5]=r12;
	m_values[6]=r20; m_values[7]=r21; m_values[8]=r22;
}

template <typename Scalar, int ROWS, int COLS>
Matrix<Scalar,ROWS,COLS>::Matrix(const Scalar& r00, const Scalar& r01, const Scalar& r02, const Scalar& r03,
                                 const Scalar& r10, const Scalar& r11, const Scalar& r12, const Scalar& r13,
                                 const Scalar& r20, const Scalar& r21, const Scalar& r22, const Scalar& r23,
                                 const Scalar& r30, const Scalar& r31, const Scalar& r32, const Scalar& r33 ) {
	assert(ROWS==4 && COLS==4); 
	m_values[ 0]=r00; m_values[ 1]=r01; m_values[ 2]=r02; m_values[ 3] = r03;
	m_values[ 4]=r10; m_values[ 5]=r11; m_values[ 6]=r12; m_values[ 7] = r13;
	m_values[ 8]=r20; m_values[ 9]=r21; m_values[10]=r22; m_values[11] = r23;
	m_values[12]=r30; m_values[13]=r31; m_values[14]=r32; m_values[15] = r33;
}

template <typename Scalar, int ROWS, int COLS>
Matrix<Scalar,ROWS,COLS>::Matrix(const Scalar& v) {
	for (int i=0; i < ROWS*COLS; ++i) {
		m_values[i] = v;
	}
}

template <typename Scalar, int ROWS, int COLS>
Matrix<Scalar,ROWS,COLS>::Matrix(const Scalar values[ROWS*COLS]) {
	memcpy(m_values, values, ROWS*COLS*sizeof(Scalar));
}

template <typename Scalar, int ROWS, int COLS>
Matrix<Scalar,ROWS,COLS>* Matrix<Scalar,ROWS,COLS>::unity() {
	Matrix<Scalar,ROWS,COLS>* result = new Matrix<Scalar,ROWS,COLS>(0);
	for (int i=0; i<std::min(ROWS,COLS); ++i) (*result)(i,i) = 1;
	return result;
}

template <typename Scalar, int ROWS, int COLS>
template <typename otherScalarType>
Matrix<Scalar,ROWS,COLS>::Matrix(const Matrix<otherScalarType,ROWS,COLS>& other) {
	operator=(other);
}

template <typename Scalar, int ROWS, int COLS>
template <typename otherScalarType>
inline Matrix<Scalar,ROWS,COLS>& Matrix<Scalar,ROWS,COLS>::operator=(const Matrix<otherScalarType,ROWS,COLS>& other) {
	for (int i=0; i < ROWS*COLS; ++i) {
		(Scalar)other.m_values[i];
	}
	return *this; 
}

template <typename Scalar, int ROWS, int COLS>
inline Matrix<Scalar,ROWS,COLS>::operator Scalar*() {
	return m_values;
}

template <typename Scalar, int ROWS, int COLS>
inline Matrix<Scalar,ROWS,COLS>::operator const Scalar*() const {
	return m_values;
}

template <typename Scalar, int ROWS, int COLS>
inline Scalar& Matrix<Scalar,ROWS,COLS>::operator()(int i, int j) {
	assert(i>=0 && i<ROWS && j>=0 && j<COLS); return m_values[i*COLS + j]; 
}

template <typename Scalar, int ROWS, int COLS>
inline const Scalar& Matrix<Scalar,ROWS,COLS>::operator()(int i, int j) const {
	assert(i>=0 && i<ROWS && j>=0 && j<COLS); return m_values[i*COLS + j]; 
}

template <typename Scalar, int ROWS, int COLS>
inline Scalar& Matrix<Scalar,ROWS,COLS>::operator()(size_t i, size_t j) {
	assert(i<ROWS && j<COLS); return m_values[i*COLS + j]; 
}

template <typename Scalar, int ROWS, int COLS>
inline const Scalar& Matrix<Scalar,ROWS,COLS>::operator()(size_t i, size_t j) const {
	assert(i<ROWS && j<COLS); return m_values[i*COLS + j];
}

template <typename Scalar, int ROWS, int COLS>
inline Vector<Scalar,COLS> Matrix<Scalar,ROWS,COLS>::getRow(int i) const {
	assert(i>=0 && i<ROWS);
	Vector<Scalar,COLS> result;
	for (int j = 0; j < COLS; ++j) {
		result[j] = (*this)(i,j);
	}
	return result;
}

template <typename Scalar, int ROWS, int COLS>
inline Vector<Scalar,ROWS> Matrix<Scalar,ROWS,COLS>::getColumn(int j) const {
	assert(j>=0 && j<COLS);
	Vector<Scalar,ROWS> result;
	for (int i = 0; i < ROWS; ++i) {
		result[i] = (*this)(i,j);
	}
	return result;
}

template <class Scalar, int ROWS, int COLS>
void Matrix<Scalar,ROWS,COLS>::setRow(int i, Vector<Scalar,COLS> row) {
	assert(i>=0 && i<ROWS);
	int offset = i*COLS;
	for (int j = 0; j < COLS; ++j) {
		m_values[offset + j] = row[j];
	}
}

template <class Scalar, int ROWS, int COLS>
void Matrix<Scalar,ROWS,COLS>::setColumn(int j, Vector<Scalar,ROWS> col) {
	assert(j>=0 && j<COLS);
	for (int i = 0; i < ROWS; ++i) {
		m_values[i*COLS + j] = col[i];
	}
}

template <typename Scalar, int ROWS, int COLS>
inline bool Matrix<Scalar,ROWS,COLS>::operator==(const Matrix<Scalar,ROWS,COLS>& other) const {
	for (int i=0; i < ROWS*COLS; ++i) {
		if( m_values[i] != other.m_values[i] ) return false;
	}
	return true; 
}

template <typename Scalar, int ROWS, int COLS>
inline bool Matrix<Scalar,ROWS,COLS>::operator!=(const Matrix<Scalar,ROWS,COLS>& other) const {
	return !(*this == other);
}

template <typename Scalar, int ROWS, int COLS>
inline Matrix<Scalar,ROWS,COLS> Matrix<Scalar,ROWS,COLS>::operator-(void) const {
	Matrix<Scalar,ROWS,COLS> v;
	for (int i=0; i < ROWS*COLS; ++i) {
		v.m_values[i] = -m_values[i];
	}
	return v; 
}

template <typename Scalar, int ROWS, int COLS>
inline Matrix<Scalar,ROWS,COLS>& Matrix<Scalar,ROWS,COLS>::operator*=(const Scalar& s) {
	for (int i=0; i < ROWS*COLS; ++i) {
		m_values[i] *= s;
	}
	return *this; 
}

template <typename Scalar, int ROWS, int COLS>
inline Matrix<Scalar,ROWS,COLS>& Matrix<Scalar,ROWS,COLS>::operator/=(const Scalar& s) {
	Scalar inv = Scalar(1)/s;
	for (int i=0; i < ROWS*COLS; ++i) {
		m_values[i] *= inv;
	}
	return *this; 
}

template <typename Scalar, int ROWS, int COLS>
inline Matrix<Scalar,ROWS,COLS> Matrix<Scalar,ROWS,COLS>::operator*(const Scalar& s) const {
	return Matrix<Scalar,ROWS,COLS>(*this) *= s;
}

template <typename Scalar, int ROWS, int COLS>
inline Matrix<Scalar,ROWS,COLS> Matrix<Scalar,ROWS,COLS>::operator/(const Scalar& s) const {
	return Matrix<Scalar,ROWS,COLS>(*this) /= s;
}

template <typename Scalar, int ROWS, int COLS>
inline Vector<Scalar,ROWS> Matrix<Scalar,ROWS,COLS>::operator*(const Vector<Scalar,COLS>& v) const {
	Vector<Scalar,COLS> result(Scalar(0));
	for (int i=0; i<ROWS; ++i) {
		for (int x=0; x<COLS; ++x) result[i] += m_values[i*COLS + x] * v[x];
	}
	return result;
}

template <typename Scalar, int ROWS, int COLS>
inline Matrix<Scalar,ROWS,COLS>& Matrix<Scalar,ROWS,COLS>::operator*=(const Matrix<Scalar,COLS,ROWS>& other) {
	Matrix<Scalar,ROWS,COLS> temp;
	Scalar t;
	for (int i=0; i<ROWS; ++i) {
		for (int j=0; j<COLS; ++j) {
			t = 0;
			for (int x=0; x<COLS; ++x) t += m_values[i*COLS + x] * other(x, j);
			temp(i,j) = t;
		}
	}

	for (int i=0; i < ROWS*COLS; ++i) {
		m_values[i] = temp[i]; 
	}
	return *this; 
}

template <typename Scalar, int ROWS, int COLS>
inline Matrix<Scalar,ROWS,COLS>& Matrix<Scalar,ROWS,COLS>::operator-=(const Matrix<Scalar,ROWS,COLS>& other) {
	for (int i=0; i < ROWS*COLS; ++i) {
		m_values[i] -= other[i];
	}
	return *this; 
}

template <typename Scalar, int ROWS, int COLS>
inline Matrix<Scalar,ROWS,COLS>& Matrix<Scalar,ROWS,COLS>::operator+=(const Matrix<Scalar,ROWS,COLS>& other) {
	for (int i=0; i < ROWS*COLS; ++i) {
		m_values[i] += other[i];
	}
	return *this; 
}

template <typename Scalar, int ROWS, int COLS>
inline Matrix<Scalar,ROWS,COLS> Matrix<Scalar,ROWS,COLS>::operator*(const Matrix<Scalar,COLS,ROWS>& _v) const {
	return Matrix<Scalar,ROWS,COLS>(*this) *= _v;
}

template <typename Scalar, int ROWS, int COLS>
inline Matrix<Scalar,ROWS, COLS> Matrix<Scalar,ROWS,COLS>::operator+(const Matrix<Scalar,ROWS,COLS>& _v) const {
	return Matrix<Scalar,ROWS, COLS>(*this) += _v;
}

template <typename Scalar, int ROWS, int COLS>
inline Matrix<Scalar,ROWS,COLS> Matrix<Scalar,ROWS,COLS>::operator-(const Matrix<Scalar,ROWS,COLS>& _v) const {
	return Matrix<Scalar,ROWS, COLS>(*this) -= _v;
}

template <typename Scalar, int ROWS, int COLS>
inline void Matrix<Scalar,ROWS,COLS>::transpose() {
	Scalar temp;
	for (int i=0; i<ROWS; ++i) {
		for (int j=i+1; j<COLS; ++j) {
			temp = m_values[i*COLS + j];
			m_values[i*COLS + j] = m_values[j*COLS + i];
			m_values[j*COLS + i] = temp;
		}
	}
}

template <typename Scalar, int ROWS, int COLS>
inline Matrix<Scalar,ROWS,COLS> Matrix<Scalar,ROWS,COLS>::getTranspose() {
	Matrix<Scalar,ROWS,COLS> copy(*this);
	copy.transpose();
	return copy;
}


/* GLOBAL OPERATORS */
template <typename Scalar, int ROWS, int COLS>
inline Vector<Scalar,ROWS> operator*(const Vector<Scalar,COLS>& v, const Matrix<Scalar, ROWS, COLS>& m) {
	Vector<Scalar,ROWS> result(Scalar(0));
	for (int i=0; i<ROWS; ++i) {
		for (int x=0; x<COLS; ++x) {
			result[i] += m(i,x) * v[x];
		}
	}
	return result;
}

template <typename Scalar, int ROWS, int COLS>
inline std::ostream& operator<<(std::ostream& os, const Matrix<Scalar,ROWS,COLS>& mat) {
	for(int i=0; i<ROWS; ++i) {
		for(int j=0; j<COLS-1; ++j) {
			os << mat(i,j) << " ";
		}
		os << mat(i,COLS-1) << (i<ROWS-1 ? "\n" : "");
	}
	return os;
}


#undef FOR_LOOP

} // Math

#endif
