#ifndef EIGENFUNCTOR_H_
#define EIGENFUNCTOR_H_

#include <functional>
#include <Eigen/Dense>
using namespace Eigen;

namespace Optimization {

template<typename _Scalar, int NX, int NY>
struct EigenFunctor {
	typedef _Scalar Scalar;
	enum {
	 InputsAtCompileTime = NX,
	 ValuesAtCompileTime = NY
	};
	
	typedef Matrix<Scalar,Dynamic,1>        InputType;
	typedef Matrix<Scalar,Dynamic,1>        ValueType;
	typedef Matrix<Scalar,Dynamic,Dynamic>  JacobianType;

	typedef std::function<void (InputType& x, ValueType& y)> Func;
	typedef std::function<void (InputType& x, JacobianType& y)> Deriv;

	const int m_inputs, m_values;
	const Func  m_func;
	const Deriv m_deriv;

	EigenFunctor(Func func, Deriv deriv);

	int inputs() const;
	int values() const;

	int operator()(InputType& x, ValueType& y) const;
	int df(InputType& x, JacobianType& y) const;
};

#include "EigenFunctor.inl"

} // Optimization

#endif /* EIGENFUNCTOR_H_ */
