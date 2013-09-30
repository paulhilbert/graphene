#include <iostream> // for graphics
#include <fstream>
using std::ofstream;

#include <limits>
using std::numeric_limits;

#include <Random/RNG.h>
using namespace Random;

#include <Math/Vector.h>
#include <Math/Data/EuclideanDistance.h>
#include <Cluster/Kmedoid.h>
using namespace Math;
using namespace Math::Data;
using namespace Cluster;

typedef double Scalar;
const int Dim = 2;
typedef Vector<Scalar, Dim> Point;
const int pointsPerCentroid = 80;

int main(int argc, char* argv[]) {
	vector<Point> centroids(324);
	for (unsigned int i=0; i<324; ++i) {
		int y = i / 18;
		int x = i % 18;
		centroids[i] = Point(Scalar(x),Scalar(y));
	}

	RNG* rng = RNG::instance();
	typename RNG::Traits<Scalar>::GenNormal gen = rng->normalGen<Scalar>();

	vector<Point> points;
	for (unsigned int c=0; c<centroids.size(); ++c) {
		points.push_back(centroids[c]);
		Scalar sigma(sqrt(1.0/8.0));
		typename RNG::Traits<Scalar>::GenNormal genX = rng->normalGen<Scalar>(centroids[c][0], sigma);
		typename RNG::Traits<Scalar>::GenNormal genY = rng->normalGen<Scalar>(centroids[c][1], sigma);
		for (int p=0; p<pointsPerCentroid; ++p) {
			points.push_back(Point(genX(), genY()));
		}
	}
	std::cout << points.size() << "\n";
	ofstream pointsOut("../matlab/pointset.dat");
	if (!pointsOut.good()) {
		std::cout << "Could not open pointset.dat for writing" << "\n";
		return 1;
	}
	for (unsigned int p=0; p<points.size(); ++p) {
		pointsOut << points[p][0] << " " << points[p][1] << "\n";
	}
	pointsOut.close();

	EuclideanDistance<Scalar,Dim> edist(points);
	Kmedoid<Scalar, Dim> kmed(edist);

	typename Kmedoid<Scalar, Dim>::ResultSteps* steps = new typename Kmedoid<Scalar, Dim>::ResultSteps();

	typename Kmedoid<Scalar, Dim>::ResultType result = kmed.compute(324, 200000, 5000, steps);

	ofstream medoidsOut("../matlab/medoids.dat");
	if (!medoidsOut.good()) {
		std::cout << "Could not open medoids.dat for writing" << "\n";
		return 1;
	}
	for (unsigned int i=0; i<result.first.size(); ++i) {
		medoidsOut << result.first[i]+1 << "\n";
	}
	medoidsOut.close();

	ofstream clusterOut("../matlab/cluster.dat");
	if (!clusterOut.good()) {
		std::cout << "Could not open cluster.dat for writing" << "\n";
		return 1;
	}
	for (unsigned int i=0; i<result.second.size(); ++i) {
		clusterOut << (result.second[i]+1) << "\n";
	}
	clusterOut.close();

	/*
	ofstream medoidStepsOut("../matlab/medoidsteps.dat");
	ofstream clusterStepsOut("../matlab/clustersteps.dat");
	if (!medoidStepsOut.good() || !clusterStepsOut.good()) {
		std::cout << "Could not open medoidsteps.dat and/or clustersteps.dat for writing" << "\n";
		return 1;
	}
	for (unsigned int i=0; i<result.first.size(); ++i) {
		for (unsigned int j=0; j<steps->size(); ++j) {
			medoidStepsOut << (*steps)[j].first[i] << " ";
		}
		medoidStepsOut << "\n";
	}
	for (unsigned int i=0; i<result.second.size(); ++i) {
		for (unsigned int j=0; j<steps->size(); ++j) {
			clusterStepsOut << (*steps)[j].second[i] << " ";
		}
		clusterStepsOut << "\n";
	}
	clusterStepsOut.close();
	medoidStepsOut.close();
	*/

	return 0;
}
