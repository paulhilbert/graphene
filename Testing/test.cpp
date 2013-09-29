#include <iostream>
#include <cstdlib>

#include <vector>
using std::vector;
#include <algorithm>

#include "../Random/RNG.h"
using namespace Random;

#include "../Math/Vector.h"
using namespace Math;

#include "Metric.h"
using namespace Testing;

int main(int argc, char* argv[]) {
	int k = atoi(argv[1]);
	vector<Vector<float,3>> points(k);

	// build vector of k random points
	RNG* rng = RNG::instance();
	std::generate(points.begin(), points.end(), [&] () { 
		return Vector<float,3>(
			rng->uniform01<float>(),
			rng->uniform01<float>(),
			rng->uniform01<float>()
		);
	});

	Metric<Vector<float,3>>::IdxTriples failed;
	Metric<Vector<float,3>>::testSubadditivity(points, [](Vector<float,3> a, Vector<float,3> b)->float { return (a-b).norm(); }, failed);
	std::for_each(failed.begin(), failed.end(), [] (Metric<Vector<float,3>>::IdxTriple t) {
		std::cout << std::get<0>(t) << "  -  " << std::get<1>(t) << "  -  " << std::get<2>(t) << "\n";
	});
}
