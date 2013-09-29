#ifndef __METRIC_H__
#define __METRIC_H__

#include <functional>

#include <vector>
#include <tuple>
using std::vector;
using std::tuple;
using std::pair;
using std::make_pair;
using std::make_tuple;

#include <limits>
using std::numeric_limits;

#include <boost/optional.hpp>
#include <boost/none.hpp>
using boost::optional;
using boost::none;

#include <IO/Log.h>
using namespace IO;

namespace Testing {

template <class Point, class Scalar=float>
struct Metric {
	typedef vector<Point> Points;
	typedef unsigned int Idx;
	typedef pair<Idx,Idx> IdxPair;
	typedef tuple<Idx,Idx,Idx> IdxTriple;
	typedef vector<IdxTriple> IdxTriples;
	typedef vector<IdxPair> IdxPairs;

	static bool test(const Points& p, std::function<Scalar (Point,Point)> metric);

	static bool testPositiveDefiniteness(const Points& p, std::function<Scalar (Point,Point)> metric, optional<IdxPairs&> failed = none);
	static bool testSymmetry(const Points& p, std::function<Scalar (Point,Point)> metric, optional<IdxPairs&> failed = none);
	static bool testSubadditivity(const Points& p, std::function<Scalar (Point,Point)> metric, optional<IdxTriples&> failed = none);
};


template <class Point, class Scalar>
bool Metric<Point,Scalar>::test(const Points& p, std::function<Scalar (Point,Point)> metric) {
	bool success = true;
	success &= testPositiveDefiniteness(p, metric);
	success &= testSymmetry(p, metric);
	success &= testSubadditivity(p, metric);
	return success;
}

template <class Point, class Scalar>
bool Metric<Point,Scalar>::testPositiveDefiniteness(const Points& p, std::function<Scalar (Point,Point)> metric, optional<IdxPairs&> failed) {
	Scalar d = 0.f;
	bool success = true;
	Console::ProgressBar pBar(20);
	for (unsigned int i=0; i<p.size(); ++i) {
		Log::info("Testing for positive definiteness...   " + pBar.poll(static_cast<float>(i) / (p.size()-1)), Console::CLEAR | Console::FLUSH);
		for (unsigned int j=0; j<p.size(); ++j) {
			d = metric(p[i],p[j]);
			if (i==j && fabs(d) > numeric_limits<Scalar>::epsilon() || i!=j && d < numeric_limits<Scalar>::epsilon()) {
				if (failed) failed.get().push_back(make_pair(i,j));
				success = false;
			}
		}
	}
	Log::info(success ? "Done (successful)." : "Done (failed).", Console::CLEAR | Console::END);
	return success;
}

template <class Point, class Scalar>
bool Metric<Point,Scalar>::testSymmetry(const Points& p, std::function<Scalar (Point,Point)> metric, optional<IdxPairs&> failed) {
	Scalar d1 = 0.f, d2 = 0.f;
	bool success = true;
	Console::ProgressBar pBar(20);
	for (unsigned int i=0; i<p.size(); ++i) {
		Log::info("Testing for symmetry...   " + pBar.poll(static_cast<float>(i) / (p.size()-1)), Console::CLEAR | Console::FLUSH);
		for (unsigned int j=0; j<p.size(); ++j) {
			d1 = metric(p[i],p[j]);
			d2 = metric(p[j],p[i]);
			if (fabs(d1 - d2) > std::numeric_limits<Scalar>::epsilon()) {
				success = false;
				if (failed) failed.get().push_back(make_pair(i,j));
			}
		}
	}
	Log::info(success ? "Done (successful)." : "Done (failed).", Console::CLEAR | Console::END);
	return success;
}

template <class Point, class Scalar>
bool Metric<Point,Scalar>::testSubadditivity(const Points& p, std::function<Scalar (Point,Point)> metric, optional<IdxTriples&> failed) {
	Scalar d1 = 0.f, d2 = 0.f, d3 = 0.f;
	bool success = true;
	Console::ProgressBar pBar(20);
	for (unsigned int i=0; i<p.size(); ++i) {
		Log::info("Testing for subadditivity...   " + pBar.poll(static_cast<float>(i) / (p.size()-1)), Console::CLEAR | Console::FLUSH);
		for (unsigned int j=0; j<p.size(); ++j) {
			for (unsigned int k=0; k<p.size(); ++k) {
				if (i==j || i==k || j==k) continue;
				d1 = metric(p[i], p[k]);
				d2 = metric(p[i], p[j]);
				d3 = metric(p[j], p[k]);
				if (d1 - (d2 + d3) > numeric_limits<Scalar>::epsilon()) {
					success = false;
					if (failed) failed.get().push_back(IdxTriple(i,j,k));
				}
			}
		}
	}
	Log::info(success ? "Done (successful)." : "Done (failed).", Console::CLEAR | Console::END);
	return success;
}

} // Testing

#endif // __METRIC_H__
