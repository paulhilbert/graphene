#ifndef KMEDOID_H_
#define KMEDOID_H_

#include <vector>
#include <set>
using std::vector;
using std::pair;

#include <memory>
using std::shared_ptr;

#include <boost/optional.hpp>
#include <boost/none.hpp>
using boost::optional;
using boost::none;

#include <Math/Data/Distance.h>
using namespace Math::Data;

#include <functional>
using namespace std::placeholders;

#include <Generic/MT.h>
#include <Generic/CachedDistanceMatrix.h>
using namespace Generic;

#include <Random/RNG.h>
using namespace Random;

#include <IO/Console.h>
#include <IO/Log.h>
using namespace IO;

#define NUMCORES 1
#ifndef NUMCORES
#define NUMCORES 1
#endif

namespace Cluster {

template <class Scalar, class Point>
class Kmedoid {
	public:
		typedef Distance<Scalar, Point> Dist;
		typedef vector<unsigned int> Medoids;
		typedef vector<unsigned int> Cluster;
		typedef vector<Scalar> Distances;
		typedef pair<Medoids,Cluster> ResultType;
		typedef vector<ResultType> ResultSteps;
		typedef typename Distance<Scalar,Point>::PointPair KeyType;

	public:
		Kmedoid(Dist& dist);
		virtual ~Kmedoid() {}

		pair<Medoids,Cluster> compute(unsigned int k, unsigned int maxIterations, unsigned int maxFailedAttempts, ResultSteps* steps = NULL, bool printBar = true, optional<Medoids> initialMedoids = none);

	protected:
		void determineRandomMedoids(unsigned int k);
		Distances determineClusterCorrespondence(CachedDistanceMatrix<Scalar, KeyType>& cache, bool printBar);
		Scalar updateClusterCorrespondence(const Distances& oldDistances, Distances& newDistances, Cluster& newCluster, unsigned int medoid, unsigned int nonmedoid, CachedDistanceMatrix<Scalar, KeyType>& cache);
		bool isPointMedoid(unsigned int point) const;

	protected:
		Distance<Scalar,Point>& m_dist;
		unsigned int m_numPoints;
		Medoids m_medoids;
		Cluster m_cluster;
		RNG* m_rng;

	protected:
		struct MatrixFiller {
				MatrixFiller(CachedDistanceMatrix<Scalar, KeyType>& cache, int minRow, int maxRow, int cols) : m_cache(cache), m_minRow(minRow), m_maxRow(maxRow), m_cols(cols), m_stop(false) {}

				void operator()();
				inline void stop() { m_stop = true; }

			protected:
				CachedDistanceMatrix<Scalar, KeyType>& m_cache;
				int m_minRow;
				int m_maxRow;
				int m_cols;
				bool m_stop;
		};
};

template <class Scalar, class Point>
Kmedoid<Scalar,Point>::Kmedoid(Dist& dist) : m_dist(dist) {
	m_rng = RNG::instance();
	m_rng->seed();
}

template <class Scalar, class Point>
typename Kmedoid<Scalar, Point>::ResultType Kmedoid<Scalar, Point>::compute(unsigned int k, unsigned int maxIterations, unsigned int maxFailedAttempts, ResultSteps* steps, bool printBar, optional<Medoids> initialMedoids) {
	unsigned int n = m_dist.getPointCount();
	if (k>=n) {
		Log::warn("k must be less than point count");
		// return all points as medoids
		Medoids m(k); for (unsigned int i=0; i<k; ++i) m[i] = i;
		Cluster c(n);
		return ResultType(m,c);
	}
	if (initialMedoids && initialMedoids.get().size() == k) {
		m_medoids = initialMedoids.get();
	} else {
		if (initialMedoids) Log::warn("Initial medoids must be a vector of size k. Using random medoids.");
		determineRandomMedoids(k);
	}

	typename CachedDistanceMatrix<Scalar, KeyType>::RetrievalFunc compute = std::bind(&Distance<Scalar,Point>::compute, &m_dist, std::_Placeholder<1>());
	CachedDistanceMatrix<Scalar, KeyType> cache(n, compute);

	vector< shared_ptr<MatrixFiller> > auxFiller;
	vector< shared_ptr<boost::thread> > auxThreads;
#if (NUMCORES > 1)
	int numRows = static_cast<int>(n) / static_cast<int>(NUMCORES-1);
	for (int i=0; i<(NUMCORES-1); ++i) {
		int start = i*numRows;
		int end = i*numRows + numRows-1; // end is inclusive
		shared_ptr<MatrixFiller> m(new MatrixFiller(cache, start, end, n));
		auxFiller.push_back(m);
		shared_ptr<boost::thread> t(new boost::thread(*m));
		auxThreads.push_back(t);
	}
#endif

	Distances distA = determineClusterCorrespondence(cache, printBar);
	Distances distB(n);
	Cluster newCluster(n);
	bool crtIsA = true;

	// vectors for storing current distances to medoids

	unsigned int iteration=0;
	unsigned int numFailedAttempts = 0;
	RNG* rng = RNG::instance();
	rng->seed();
	typename RNG::Traits<unsigned int>::GenAB rndPoint = rng->uniformABGen<unsigned int>(0, n);
	typename RNG::Traits<unsigned int>::GenAB rndMedoid = rng->uniformABGen<unsigned int>(0, m_medoids.size());
	if (steps) {
		steps->clear();
		steps->push_back(ResultType(m_medoids, m_cluster));
	}
	Console::ProgressBar pBar(20);
	while (iteration < maxIterations) {
		if (printBar) {
			Log::info(string("k-medoid clustering...  ") + pBar.poll(static_cast<float>(iteration)/(maxIterations-1)), Console::CLEAR | Console::FLUSH);
		}
		++iteration;
		if (numFailedAttempts > maxFailedAttempts) {
			Log::info("Stopping because of too many failed attempts in iteration.");
			break;
		}
		// randomly chose a medoid and a non-medoid
		unsigned int medoid = rndMedoid();
		unsigned int nonmedoid;
		bool alreadyIsMedoid = true;
		while (alreadyIsMedoid) { nonmedoid = rndPoint(); alreadyIsMedoid = isPointMedoid(nonmedoid); }

		Scalar diff;
		if (crtIsA) diff = updateClusterCorrespondence(distA, distB, newCluster, medoid, nonmedoid, cache);
		else diff = updateClusterCorrespondence(distB, distA, newCluster, medoid, nonmedoid, cache);

		if (diff < 0.0) {
			// if we didn't decrease overall distance sum, we don't want to switch distance arrays
			m_cluster = newCluster;
			m_medoids[medoid] = nonmedoid;
			crtIsA = !crtIsA;
			numFailedAttempts = 0;
			if (steps) steps->push_back(ResultType(m_medoids, m_cluster));
		} else { ++numFailedAttempts; }
	}
	if (printBar) std::cout << "\n";
	for (unsigned int i=0; i<auxFiller.size(); ++i) auxFiller[i]->stop();
	for (unsigned int i=0; i<auxThreads.size(); ++i) auxThreads[i]->join();
	return ResultType(m_medoids, m_cluster);
}

template <class Scalar, class Point>
void Kmedoid<Scalar, Point>::determineRandomMedoids(unsigned int k) {
	unsigned int n = m_dist.getPointCount();
	Medoids range = Medoids(n);
	m_medoids = Medoids(k);
	for (unsigned int i=0; i<n; ++i) range[i] = i;
	for (unsigned int i=0; i<k; ++i) {
		unsigned int e = m_rng->uniformAB<unsigned int>(i,n);
		m_medoids[i] = range[e];
		range[e] = range[i];
	}
}

template <class Scalar, class Point>
typename Kmedoid<Scalar, Point>::Distances Kmedoid<Scalar, Point>::determineClusterCorrespondence(CachedDistanceMatrix<Scalar, KeyType>& cache, bool printBar) {
	unsigned int n = m_dist.getPointCount();
	m_cluster = Cluster(n, 0);
	Distances minDist(n);

	int totalIterations = m_medoids.size() * n;
	Console::ProgressBar pBar(20);
	for (unsigned int p=0; p<n; ++p) {
		if (printBar && (p%100) == 0) {
			float progress = static_cast<float>(p) / (totalIterations-1);
			Log::info(string("Initial correspondence...  ")+pBar.poll(progress), Console::CLEAR | Console::FLUSH);
		}
		minDist[p] = (p==m_medoids[0]) ? Scalar(0) : cache.get(KeyType(p, m_medoids[0]));
	}
	for (unsigned int c=1; c<m_medoids.size(); ++c) {
		for (unsigned int p=0; p<n; ++p) {
			if (printBar && (p%100) == 0) {
				float progress = static_cast<float>(c*n+p) / (totalIterations-1);
				Log::info(string("Initial correspondence...  ") + pBar.poll(progress), Console::CLEAR | Console::FLUSH);
			}
			Scalar dist = (p==m_medoids[c]) ? Scalar(0) : cache.get(KeyType(p, m_medoids[c]));
			if (dist < minDist[p]) {
				m_cluster[p] = c;
				minDist[p] = dist;
			}
		}
	}

	return minDist;
}

template <class Scalar, class Point>
Scalar Kmedoid<Scalar, Point>::updateClusterCorrespondence(const Distances& oldDistances, Distances& newDistances, Cluster& newCluster, unsigned int medoid, unsigned int nonmedoid, CachedDistanceMatrix<Scalar, KeyType>& cache) {
	unsigned int n = m_dist.getPointCount();
	Scalar diff = Scalar(0);
	newDistances = oldDistances;
	newCluster = m_cluster;
	for (unsigned int p=0; p<n; ++p) {
		// if old medoid is not changed, we only have to check if new medoid is closer
		Scalar dist = (p==nonmedoid) ? Scalar(0) : cache.get(KeyType(p, nonmedoid));
		if (m_cluster[p] != medoid) {
			if (dist < oldDistances[p]) {
				newDistances[p] = dist;
				newCluster[p] = nonmedoid;
				diff += dist - oldDistances[p];
			}
		} else {
			// if old medoid is now changed, we have to check all medoids again
			newDistances[p] = numeric_limits<Scalar>::max();
			for (unsigned int c=0; c<m_medoids.size(); ++c) {
				if (c==medoid) {
					if (dist < newDistances[p]) { newDistances[p] = dist; newCluster[p] = m_cluster[p]; }
				} else {
					Scalar crtDist = (p==m_medoids[c]) ? Scalar(0) : cache.get(KeyType(p, m_medoids[c]));
					if (crtDist < newDistances[p]) { newDistances[p] = crtDist; newCluster[p] = c; }
				}
			}
			// at this point newDistances[p] should be the minimal distance to all *new* medoids
			diff += newDistances[p] - oldDistances[p];
		}
	}

	return diff;
}

template <class Scalar, class Point>
inline bool Kmedoid<Scalar,Point>::isPointMedoid(unsigned int point) const {
	for (unsigned int i=0; i<m_medoids.size(); ++i) {
		if (m_medoids[i] == point) return true;
	}
	return false;
}

template <class Scalar, class Point>
void Kmedoid<Scalar,Point>::MatrixFiller::operator()() {
	std::cout << "spasss" << "\n";
	for (int i=m_minRow; i<=m_maxRow; ++i) {
		for (int j=i+1; j<m_cols; ++j) {
			if (m_stop) return;
			m_cache.get(KeyType(i,j));
		}
	}
}

} // Cluster

#endif /* KMEDOID_H_ */
