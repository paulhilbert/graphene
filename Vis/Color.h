#ifndef VISCOLOR_H
#define VISCOLOR_H

#include <Eigen/Dense>
#include <boost/strong_typedef.hpp>

namespace Vis {

//template <class Scalar=float, int Dim=3>
//using Color = Eigen::Matrix<Scalar, Dim, 1>;


/*
// the following structs are c&p (as well as slightly modified)
// versions of the BOOST_STRONG_TYPEDEF macro

template <class Scalar=float>
struct RGB : public Color<Scalar, 3> {
	typedef Color<Scalar,3> Base;
	typedef Scalar ScalarType;
	Base t;
	explicit RGB(const Base t_) : t(t_) {}
	RGB() { }
	RGB(const RGB<Scalar> & t_) : t(t_.t) {}
	RGB<Scalar>& operator=(const RGB<Scalar> & rhs) { t = rhs.t; return *this;}
	RGB<Scalar>& operator=(const Base & rhs) { t = rhs; return *this;}
	operator const Base & () const {return t; }
	operator Base & () { return t; }
	bool operator==(const RGB<Scalar> & rhs) const { return t == rhs.t; }
	bool operator<(const RGB<Scalar> & rhs) const { return t < rhs.t; }
};

template <class Scalar=float>
struct RGBA : public Color<Scalar, 4> {
	typedef Color<Scalar,4> Base;
	typedef Scalar ScalarType;
	Base t;
	explicit RGBA(const Base t_) : t(t_) {}
	RGBA() { }
	RGBA(const RGBA<Scalar> & t_) : t(t_.t) {}
	RGBA<Scalar>& operator=(const RGBA<Scalar> & rhs) { t = rhs.t; return *this;}
	RGBA<Scalar>& operator=(const Base & rhs) { t = rhs; return *this;}
	operator const Base & () const {return t; }
	operator Base & () { return t; }
	bool operator==(const RGBA<Scalar> & rhs) const { return t == rhs.t; }
	bool operator<(const RGBA<Scalar> & rhs) const { return t < rhs.t; }
};

template <class Scalar=float>
struct HSV : public Color<Scalar, 3> {
	typedef Color<Scalar,3> Base;
	typedef Scalar ScalarType;
	Base t;
	explicit HSV(const Base t_) : t(t_) {}
	HSV() { }
	HSV(const HSV<Scalar> & t_) : t(t_.t) {}
	HSV<Scalar>& operator=(const HSV<Scalar> & rhs) { t = rhs.t; return *this;}
	HSV<Scalar>& operator=(const Base & rhs) { t = rhs; return *this;}
	operator const Base & () const {return t; }
	operator Base & () { return t; }
	bool operator==(const HSV<Scalar> & rhs) const { return t == rhs.t; }
	bool operator<(const HSV<Scalar> & rhs) const { return t < rhs.t; }
};

template <class Scalar=float>
struct HSVA : public Color<Scalar, 4> {
	typedef Color<Scalar,4> Base;
	typedef Scalar ScalarType;
	Base t;
	explicit HSVA(const Base t_) : t(t_) {}
	HSVA() { }
	HSVA(const HSVA<Scalar> & t_) : t(t_.t) {}
	HSVA<Scalar>& operator=(const HSVA<Scalar> & rhs) { t = rhs.t; return *this;}
	HSVA<Scalar>& operator=(const Base & rhs) { t = rhs; return *this;}
	operator const Base & () const {return t; }
	operator Base & () { return t; }
	bool operator==(const HSVA<Scalar> & rhs) const { return t == rhs.t; }
	bool operator<(const HSVA<Scalar> & rhs) const { return t < rhs.t; }
};
*/

typedef Eigen::Matrix<float, 3, 1> RGB;
typedef Eigen::Matrix<float, 3, 1> HSV;
typedef Eigen::Matrix<float, 4, 1> RGBA;
typedef Eigen::Matrix<float, 4, 1> HSVA;
/*
template <class Scalar=float>
using RGB  = Color<Scalar, 3>;

template <class Scalar=float>
using HSV  = Color<Scalar, 3>;

template <class Scalar=float>
using RGBA = Color<Scalar, 4>;

template <class Scalar=float>
using HSVA = Color<Scalar, 4>;
*/

} // Vis

#endif // VISCOLOR_H
