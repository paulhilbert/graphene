#ifndef MINIMIZE_H_
#define MINIMIZE_H_

#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
using namespace Eigen;

#include "EigenFunctor.h"

namespace Optimization {

template<class Scalar, int NX, int NY>
struct Minimize {
	typedef EigenFunctor<Scalar,NX,NY>             Functor;
	typedef typename Functor::Func                 Func;
	typedef typename Functor::Deriv                Deriv;
	typedef typename Functor::InputType            Inputs;
	typedef typename Functor::ValueType            Values;
	typedef typename Functor::JacobianType         Jacobian;
	typedef HybridNonLinearSolver<Functor,Scalar>  Solver;
	
	static Inputs solve(const Func& func, const Deriv& deriv, const Inputs& xStart) {
		typename Solver::FVectorType x(xStart);
		Functor functor(func, deriv);
		Solver solver(functor);
		solver.solve(x);
		return x;
	}
};

} // Optimization

#endif /* MINIMIZE_H_ */
