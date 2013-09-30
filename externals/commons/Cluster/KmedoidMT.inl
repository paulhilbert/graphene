template <class TDistance>
KmedoidMT<TDistance>::KmedoidMT(const Points& points, uint numCores) : m_points(points), m_numCores(numCores) {
	m_rng = RNG::instance();
	m_rng->seed();
}

template <class TDistance>
KmedoidMT<TDistance>::~KmedoidMT() {
}

template <class TDistance>
ClusterResult KmedoidMT<TDistance>::compute(uint k, uint maxIterations, uint maxFailedAttempts, DistanceFactory factory, optional<Medoids> initialMedoids) {
	uint n = m_points.size();
	// check k
	if (k>=n) {
		Log::warn("k must be less than point count");
		// return all points as medoids
		Medoids m(n); for (uint i=0; i<n; ++i) m[i] = i;
		Cluster c(n);
		return ClusterResult(m,c);
	}

	m_cache = std::shared_ptr<Cache>(new Cache(n));

	/*
	// prefill distance cache if wanted
	if (initialFill) {
//		scatterFill(initialFill.get(), cache, factory);
	}
	*/

	if (initialMedoids && initialMedoids.get().size() == k) {
		m_medoids = initialMedoids.get();
	} else {
		if (initialMedoids) Log::warn("Initial medoids must be a vector of size k. Using random medoids.");
		determineRandomMedoids(k);
	}

	// first step: find closest medoid for each point and store distances
	Scalar* distA = new Scalar[n];
	Scalar* distB = new Scalar[n];
	determineClusterCorrespondence(distA, factory);

	// prepare for iteration
	Cluster newCluster(n);
	typename RNG::Traits<unsigned int>::GenAB rndPoint = m_rng->uniformABGen<unsigned int>(0, n);
	typename RNG::Traits<unsigned int>::GenAB rndMedoid = m_rng->uniformABGen<unsigned int>(0, m_medoids.size());
	Console::ProgressBar pBar(20);

	// iterate: sample new medoid, update distances and clusters 
	//          then check for improvement
	unsigned int iteration=0;
	unsigned int numFailedAttempts = 0;
	bool crtIsA = true;
	while (iteration < maxIterations) {
		Log::info(string("k-medoid clustering...  ") + pBar.poll(static_cast<float>(iteration)/(maxIterations-1)), Console::CLEAR | Console::FLUSH);
		if (numFailedAttempts > maxFailedAttempts) {
			Log::info("Stopping because of too many failed attempts in iteration.");
			break;
		}
		// randomly chose a medoid and a non-medoid
		unsigned int medoid = rndMedoid();
		unsigned int nonmedoid;
		bool alreadyIsMedoid = true;
		while (alreadyIsMedoid) { nonmedoid = rndPoint(); alreadyIsMedoid = std::find(m_medoids.begin(), m_medoids.end(), nonmedoid) != m_medoids.end(); }

		Scalar diff;
		if (crtIsA) diff = updateClusterCorrespondence(distA, distB, newCluster, medoid, nonmedoid, factory);
		else diff = updateClusterCorrespondence(distB, distA, newCluster, medoid, nonmedoid, factory);

		if (diff < 0.0) {
			// if we lowered overall distance sum, we want to switch distance arrays
			m_cluster = newCluster;
			m_medoids[medoid] = nonmedoid;
			crtIsA = !crtIsA;
			numFailedAttempts = 0;
		} else { ++numFailedAttempts; }
		++iteration;
	}
	Log::info("k-medoid clustering... Done.", Console::CLEAR | Console::END | Console::FLUSH);


	// tidy up
	m_cache.reset();
	delete [] distA;
	delete [] distB;

	return ClusterResult(m_medoids, m_cluster);
}

template <class TDistance>
void KmedoidMT<TDistance>::determineRandomMedoids(uint k) {
	uint n = m_points.size();
	m_medoids = Medoids(n);
	std::iota(m_medoids.begin(), m_medoids.end(), 0);
	std::random_shuffle(m_medoids.begin(), m_medoids.end());
	m_medoids.resize(k);
}

template <class TDistance>
void KmedoidMT<TDistance>::determineClusterCorrespondence(Scalar* minDists, DistanceFactory factory) {
	uint n = m_points.size();
	uint c = m_medoids.size();
	m_cluster = Cluster(n, 0);
	Scalar* dists = new Scalar[n];

	Console::ProgressBar pBar(20);
	float progress = 0.f;
	Log::info(string("Initial correspondence...  ")+pBar.poll(progress), Console::CLEAR | Console::FLUSH);
	medoidToPointsDist(m_medoids[0], minDists, factory);
	progress = static_cast<float>(1) / (c-1);
	Log::info(string("Initial correspondence...  ")+pBar.poll(progress), Console::CLEAR | Console::FLUSH);
	for (uint m=1; m<c; ++m) {
		medoidToPointsDist(m_medoids[m], dists, factory);
		for (uint p=0; p<n; ++p) {
			if (dists[p] < minDists[p]) {
				m_cluster[p] = m;
				minDists[p] = dists[p];
			}
		}
		progress = static_cast<float>(m) / (c-1);
		Log::info(string("Initial correspondence...  ")+pBar.poll(progress), Console::CLEAR | Console::FLUSH);
	}

	Log::info(string("Initial correspondence... Done."), Console::CLEAR | Console::FLUSH | Console::END);
}

template <class TDistance>
typename KmedoidMT<TDistance>::Scalar KmedoidMT<TDistance>::updateClusterCorrespondence(Scalar* oldDistances, Scalar* newDistances, Cluster& newCluster, uint medoid, uint nonmedoid, DistanceFactory factory) {
	unsigned int n = m_points.size();
	Scalar diff = Scalar(0);

	// compute distance of new medoid to all points
	medoidToPointsDist(nonmedoid, newDistances, factory);

	// check each point if new medoid is closer and compute overall distance difference
	for (unsigned int p=0; p<n; ++p) {
		// if old medoid is not changed, we only have to check if new medoid is closer
		if (m_cluster[p] != medoid) {
			if (newDistances[p] < oldDistances[p]) {
				newCluster[p] = nonmedoid;
				diff += newDistances[p] - oldDistances[p];
			} else {
				// old distances are still best new distances
				newDistances[p] = oldDistances[p];
			}
		} else {
			// if old medoid is now changed, we have to check all medoids again
			Scalar dist = newDistances[p];
			newDistances[p] = std::numeric_limits<Scalar>::max();
			for (unsigned int c=0; c<m_medoids.size(); ++c) {
				if (c==medoid) {
					if (dist < newDistances[p]) { newDistances[p] = dist; newCluster[p] = m_cluster[p]; }
				} else {
					uint x = std::min(p, m_medoids[c]), y = std::max(p, m_medoids[c]);
					Scalar crtDist = (p==m_medoids[c]) ? Scalar(0) : *((*m_cache)(x, y));
					if (crtDist < newDistances[p]) { newDistances[p] = crtDist; newCluster[p] = c; }
				}
			}
			// at this point newDistances[p] should be the minimal distance to all *new* medoids
			diff += newDistances[p] - oldDistances[p];
		}
	}

	return diff;
}

template <class TDistance>
inline void KmedoidMT<TDistance>::medoidToPointsDist(uint medoid, Scalar* dist, DistanceFactory factory) {
	std::vector<std::shared_ptr<std::thread>> workers;
	uint blockSize = m_points.size() / m_numCores;
	for (uint i=0; i<m_numCores; ++i) {
		uint begin = i*blockSize;
		uint end = (i==m_numCores-1) ? m_points.size() : (i+1)*blockSize;
		std::shared_ptr<std::thread> t(new std::thread(&KmedoidMT<TDistance>::medoidToPointRangeDist, this, medoid, begin, end, &(dist[begin]), factory));
		workers.push_back(t);
	}

	for (uint i=0; i<workers.size(); ++i) {
		workers[i]->join();
	}
	workers.clear();
	// write to cache
	for (uint i=0; i<m_points.size(); ++i) {
		if (i==medoid) continue;
		uint x = std::min(i,medoid), y = std::max(i,medoid);
		*((*m_cache)(x,y)) = dist[i];
	}
}

template <class TDistance>
inline void KmedoidMT<TDistance>::medoidToPointRangeDist(uint medoid, uint begin, uint end, Scalar* dist, DistanceFactory factory) {
	std::shared_ptr<TDistance> distance = factory();
	for (uint i=begin; i<end; ++i) {
		if (i==medoid) { *(dist++) = Scalar(0); continue; }
		Scalar d = (i<medoid) ? *((*m_cache)(i,medoid)) : *((*m_cache)(medoid,i));
		*(dist++) = d > 0 ? d : distance->compute(m_points[medoid], m_points[i]);
	}
}

/*
template <class TDistance>
void KmedoidMT<TDistance>::scatterFill(float ratio, KmedoidMTCache<Scalar>& cache, DistanceFactory factory) {
	uint n = m_points.size();
	float fill = ratio;
	if (fill < 0.f || fill > 1.f) {
			Log::warn("Initial fill ratio outside of allowed interval [0,1]. Setting ratio to boundary.");
			if (fill < 0.f) fill = 0.f; if (fill > 1.f) fill = 1.f;
		}
	// fillCount is count of values to compute
	uint fullCount = (n*(n+1))/2 - n;
	uint fillCount = static_cast<uint>(fill * fullCount);
	Log::info("Scatter-precomputing "+lexical_cast<std::string>(fillCount)+" of "+lexical_cast<std::string>(fullCount)+" distances...", Console::FLUSH);
	// compute all index-pairs, shuffle and take first fillCount pairs
	std::vector<Key> candidates;
	for (uint i=0; i<n; ++i) for (uint j=i+1; j<n; ++j) candidates.push_back(Key(i,j));
	std::random_shuffle(candidates.begin(), candidates.end());
	candidates.resize(fillCount);

	uint blockCount = candidates.size() / m_numCores;
	typename std::vector<Key>::iterator begin = candidates.begin();
	typename std::vector<Key>::iterator end;
	std::vector<std::shared_ptr<std::thread>> workers(m_numCores);
	for (uint i=0; i<m_numCores; ++i) {
		if (i == (m_numCores - 1)) { end = candidates.end(); }
		else { end = begin; std::advance(end, blockCount); }
		std::function<void (void)> func = std::bind(&KmedoidMT<TDistance>::scatterFillBlock, this, begin, end, cache, factory);
		workers[i] = std::shared_ptr<std::thread>(new std::thread(func));
		begin = end;
	}
	for (uint i=0; i<m_numCores; ++i) {
		workers[i]->join();
	}
	Log::info("Completed scatter-precomputation.", Console::END | Console::FLUSH | Console::CLEAR);
}

template <class TDistance>
void KmedoidMT<TDistance>::scatterFillBlock(typename std::vector<Key>::iterator begin, typename std::vector<Key>::iterator end, KmedoidMTCache<Scalar>& cache, DistanceFactory factory) {
	std::shared_ptr<TDistance> dist = factory(m_points);
	for (auto it = begin; it != end; ++it) {
		std::cout << it->first << " " << it->second << "\n";
		*(cache(it->first, it->second)) = dist->compute(*it);
	}
}
*/
