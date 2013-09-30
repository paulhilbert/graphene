#ifndef KMEDOIDMT_H_
#define KMEDOIDMT_H_

#include <vector>
#include <algorithm>
#include <functional>
#include <memory>
#include <thread>
#include <limits>
using namespace std::placeholders;

#include <boost/optional.hpp>
#include <boost/none.hpp>
using boost::optional;
using boost::none;

#include <Random/RNG.h>
using namespace Random;

#include <IO/Console.h>
#include <IO/Log.h>
using namespace IO;

#include "KmedoidMTCache.h"


namespace Cluster {

// typedefs
using Medoids = std::vector<unsigned int>;
using Cluster = std::vector<unsigned int>;
using ClusterResult = std::pair<Medoids,Cluster>;


// class def
template <class TDistance>
class KmedoidMT {
	public:
		typedef typename TDistance::Scalar Scalar;
		typedef typename TDistance::Point  Point;
		typedef std::vector<Scalar> Distances;
		typedef std::vector<Point> Points;
		typedef KmedoidMTCache<Scalar> Cache;
		//typedef typename TDistance::PointPair Key;
		typedef std::function<std::shared_ptr<TDistance> ()> DistanceFactory;
		//typedef vector<ResultType> ResultSteps;

	protected:
		typedef unsigned int uint;

	public:
		KmedoidMT(const Points& points, unsigned int numCores = 1);
		virtual ~KmedoidMT();

		pair<Medoids,Cluster> compute(uint k, uint maxIterations, uint maxFailedAttempts, DistanceFactory factory, optional<Medoids> initialMedoids = none);

	protected:
		void   determineRandomMedoids(unsigned int k);
		void   determineClusterCorrespondence(Scalar* minDists, DistanceFactory factory);
		Scalar updateClusterCorrespondence(Scalar* oldDistances, Scalar* newDistances, Cluster& newCluster, uint medoid, uint nonmedoid, DistanceFactory factory);
		void   medoidToPointsDist(uint medoid, Scalar* dist, DistanceFactory factory);
		void   medoidToPointRangeDist(uint medoid, uint begin, uint end, Scalar* dist, DistanceFactory factory);
		/*
		void scatterFill(float ratio, KmedoidMTCache<Scalar>& cache, DistanceFactory factory);
		void scatterFillBlock(typename std::vector<Key>::iterator begin, typename std::vector<Key>::iterator end, KmedoidMTCache<Scalar>& cache, DistanceFactory factory);
		*/

	protected:
		const Points& m_points;
		Medoids m_medoids;
		Cluster m_cluster;
		RNG* m_rng;
		std::shared_ptr<Cache> m_cache;
		unsigned int m_numCores;
};

#include "KmedoidMT.inl"

} // Cluster

#endif /* KMEDOIDMT_H_ */
