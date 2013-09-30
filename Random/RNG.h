#ifndef RNG_H_
#define RNG_H_

#include <ctime> // for time seeding

#ifdef USE_BOOST_RANDOM

#include <boost/random.hpp>
using boost::mt19937;
using boost::uniform_01;
using boost::uniform_int;
using boost::uniform_real;
using boost::geometric_distribution;
using boost::normal_distribution;
using boost::variate_generator;
#include <boost/type_traits/is_floating_point.hpp> // for dispatching uniform_AB into
#include <boost/mpl/if.hpp>                        // uniform_real or uniform_int
using boost::is_floating_point;
using boost::true_type;
using boost::false_type;
using boost::mpl::if_;

#else

#include <cstdio>
#include <cstdlib>

#endif

#include <functional>
#include "../Optimization/Minimize.h"
using namespace Optimization;

namespace Random {

class RNG {
#ifdef USE_BOOST_RANDOM
	public:
		typedef mt19937 Generator;
		template <class T>
		struct Traits {
			// uniform_01
			typedef variate_generator< Generator&, uniform_01<T> > Gen01;
			// uniform_real / uniform_int
			typedef typename boost::mpl::if_< is_floating_point<T>, uniform_real<T>, uniform_int<T> >::type DistAB;
			typedef variate_generator< Generator&, DistAB > GenAB;
			// geometric
			typedef variate_generator< Generator&, geometric_distribution<T> > GenGeom;
			// normal
			typedef variate_generator< Generator&, normal_distribution<T> > GenNormal;

			static T getUpperBound(T original, true_type is_float) { return original; }
			static T getUpperBound(T original, false_type is_float) { return original-1; }
		};
#endif

	public:
		static RNG* instance() {
			if (!m_instance) m_instance = new RNG();
			return m_instance;
		}
		static void destroy() {
			delete m_instance;
			m_instance = 0;
		}

		void seed();
		void seed(unsigned int s);

// Direct
		template <class T>
		T uniform01();
		template <class T>
		T uniformAB(T a, T b);
#ifdef USE_BOOST_RANDOM
		template <class T>
		T geometric(float p);
		template <class T>
		T normal(T mean = 0.f, T sigma = 1.f);

// Generators
		template <class T>
		typename Traits<T>::Gen01 uniform01Gen();
		template <class T>
		typename Traits<T>::GenAB uniformABGen(T a, T b);
		template <class T>
		typename Traits<T>::GenGeom geometricGen(float p);
		template <class T>
		typename Traits<T>::GenNormal normalGen(T mean = 0.f, T sigma = 1.f);
		template <class T>
		typename Traits<T>::GenNormal normalGen(const T& mean, const T& funcX, const T& funcY);
#endif

	protected:
		RNG() { seed(); }
		~RNG() {}

		// loser functions (e.g. never defined)
		RNG(const RNG& other);
		RNG& operator=(const RNG& other);

	protected:
#ifdef USE_BOOST_RANDOM
		Generator m_generator;
#endif
		static RNG* m_instance;
};




inline void RNG::seed() {
#ifdef DETERMINISTIC
	seed(0);
#else
	seed(static_cast<unsigned int>(time(0)));
#endif
}
inline void RNG::seed(unsigned int s) {
#ifdef RNG_FIXED_SEED
	s = RNG_FIXED_SEED;
#endif

#ifdef USE_BOOST_RANDOM
	m_generator.seed(s);
#else
	srand(s);
#endif
}


// Direct

template <class T>
inline T RNG::uniform01() {
#ifdef USE_BOOST_RANDOM
	typename Traits<T>::Gen01 gen = uniform01Gen<T>();
	return gen();
#else
	return static_cast<T>(rand()) / RAND_MAX;
#endif
}
template <class T>
inline T RNG::uniformAB(T a, T b) {
#ifdef USE_BOOST_RANDOM
	typename Traits<T>::GenAB gen = uniformABGen<T>(a, b);
	return gen();
#else
	return static_cast<T>((static_cast<double>(rand()) / RAND_MAX) * (b-a) + a);
#endif
}

#ifdef USE_BOOST_RANDOM
template <class T>
inline T RNG::geometric(float p) {
	typename Traits<T>::GenGeom gen = geometricGen<T>(p);
	return gen();
}

template <class T>
inline T RNG::normal(T mean, T sigma) {
	typename Traits<T>::GenNormal gen = normalGen<T>(mean, sigma);
	return gen();
}


// Generators

template <class T>
inline typename RNG::Traits<T>::Gen01 RNG::uniform01Gen() {
	uniform_01<T> dist;
	return typename Traits<T>::Gen01(m_generator, dist);
}
template <class T>
inline typename RNG::Traits<T>::GenAB RNG::uniformABGen(T a, T b) {
	// uniform_real gives [a,b) and uniform_int [a,b];
	// getUpperBound(b, ...) is b-1 for is_floating_point<T> == false_type
	typename Traits<T>::DistAB dist( a, Traits<T>::getUpperBound(b, is_floating_point<T>()) );
	return typename Traits<T>::GenAB(m_generator, dist);
}
template <class T>
inline typename RNG::Traits<T>::GenGeom RNG::geometricGen(float p) {
	geometric_distribution<T> dist(p);
	return typename Traits<T>::GenGeom(m_generator, dist);
}

template <class T>
inline typename RNG::Traits<T>::GenNormal RNG::normalGen(T mean, T sigma) {
	normal_distribution<T> dist(mean, sigma);
	return typename Traits<T>::GenNormal(m_generator, dist);
}

template <class T>
inline typename RNG::Traits<T>::GenNormal RNG::normalGen(const T& mean, const T& funcX, const T& funcY) {
	typedef Minimize<T,1,1> Minimizer;
	typedef typename Minimizer::Func Func;
	typedef typename Minimizer::Deriv Deriv;
	typedef typename Minimizer::Inputs Inputs;
	typedef typename Minimizer::Values Values;
	typedef typename Minimizer::Jacobian Jacobian;

	T y_times_sqrt_2pi = funcY * sqrt(2*M_PI);
	T shifted_x_sqr = (funcX - mean)*(funcX - mean);
	Func func = [&] (Inputs& x, Values& y) {
		y(0,0) = shifted_x_sqr + 2.f*x(0,0)*x(0,0) * log(x(0,0)*y_times_sqrt_2pi);
	};
	Deriv deriv = [&] (Inputs& x, Jacobian& y) {
		y(0,0) = x(0,0) * (T(4.0) * log(x(0,0) * y_times_sqrt_2pi) + T(2.0));
	};
	// start value
	Inputs sigmaStart(1); // 1 is size here
	sigmaStart(0,0) = mean + T(1.0);
	// solve
	T sigma = Minimizer::solve(func, deriv, sigmaStart)(0,0);
	normal_distribution<T> dist(mean, sigma);
	return typename Traits<T>::GenNormal(m_generator, dist);
}
#endif




} // Random



#endif /* RNG_H_ */
