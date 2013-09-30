#ifndef DISTANCES_H
#define DISTANCES_H

#include <limits>
#include <functional>
#include <vector>
using std::vector;

#include "Vector.h"

namespace Math {

class Distances {
	public:
		template <class Scalar, int Dim>
		static Scalar euclidean(const Vector<Scalar, Dim>& x, const Vector<Scalar, Dim>& y);
		template <class Scalar, int Dim>
		static Scalar chiSquare(const Vector<Scalar, Dim>& x, const Vector<Scalar, Dim>& y);
		template <class Scalar, int Dim>
		static Scalar pairwiseMin(const Vector<Scalar, Dim>& x, const Vector<Scalar, Dim>& y);

		template <class Scalar>
		static Scalar euclidean(const vector<Scalar>& x, const vector<Scalar>& y);
		template <class Scalar>
		static Scalar chiSquare(const vector<Scalar>& x, const vector<Scalar>& y);
		template <class Scalar>
		static Scalar pairwiseMin(const vector<Scalar>& x, const vector<Scalar>& y);

		template <class Scalar>
		static Scalar transformDist(const vector<Scalar>& x, const vector<Scalar>& y, std::function<vector<Scalar> (const vector<Scalar>&)> transform);
};

template <class Scalar, int Dim>
Scalar Distances::euclidean(const Vector<Scalar, Dim>& x, const Vector<Scalar, Dim>& y) {
	return (x-y).norm();
}

template <class Scalar, int Dim>
Scalar Distances::chiSquare(const Vector<Scalar, Dim>& x, const Vector<Scalar, Dim>& y) {
	Scalar dist = 0.0;
	for (int i = 0; i < Dim; ++i) {
		Scalar t = x[i] - y[i];
		Scalar u = x[i] + y[i];
		dist += (u < std::numeric_limits<Scalar>::epsilon()) ? 0.0 : t*t / u;
	}
	return dist;
}

template <class Scalar, int Dim>
Scalar Distances::pairwiseMin(const Vector<Scalar, Dim>& x, const Vector<Scalar, Dim>& y) {
	Scalar dist = 0.0;
	for (int i = 0; i < Dim; ++i) {
		dist += x[i] < y[i] ? x[i] : y[i];
	}
	return Scalar(1) - dist;
}

template <class Scalar>
Scalar Distances::euclidean(const vector<Scalar>& x, const vector<Scalar>& y) {
	assert(x.size() == y.size());
	Scalar dist = 0.0;
	for (unsigned int i = 0; i < x.size(); ++i) {
		dist += (x[i]-y[i])*(x[i]-y[i]);
	}
	return sqrt(dist);
}

template <class Scalar>
Scalar Distances::chiSquare(const vector<Scalar>& x, const vector<Scalar>& y) {
	assert(x.size() == y.size());
	Scalar dist = 0.0;
	for (unsigned int i = 0; i < x.size(); ++i) {
		Scalar u = x[i] + y[i];
		if (fabs(u) < std::numeric_limits<Scalar>::epsilon()) continue;
		Scalar t = x[i] - y[i];
		dist += t*t / u;
	}
	return dist;
}

template <class Scalar>
Scalar Distances::pairwiseMin(const vector<Scalar>& x, const vector<Scalar>& y) {
	assert(x.size() == y.size());
	Scalar dist = 0.0;
	for (unsigned int i = 0; i < x.size(); ++i) {
		dist += x[i]<y[i] ? x[i] : y[i];
	}
	return Scalar(1) - dist;
}

template <class Scalar>
Scalar Distances::transformDist(const vector<Scalar>& x, const vector<Scalar>& y, std::function<vector<Scalar> (const vector<Scalar>&)> transform) {
	return euclidean(x,transform(y));
}

} // Math

#endif // DISTANCES_H
