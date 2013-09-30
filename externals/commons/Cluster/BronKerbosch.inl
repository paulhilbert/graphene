template <class HasEdge>
BKResult bronKerbosch(const BKSet& G, const HasEdge& hasEdge) {
	BKNeigh N (G.size());
	for (auto v0 : G) {
		for (auto v1 : G) {
			if (v0 != v1 && hasEdge(v0,v1)) N[v0].insert(v1);
		}
	}
	BKSet R,X;
	BKResult result;

	bronKerbosch1(R, G, X, N, result);

	// sort by descending clique size
	std::sort(result.begin(), result.end(), [] (const BKSet& a, const BKSet& b) { return a.size() > b.size(); });

	// make disjoint
	for (unsigned int i=1; i < result.size();) {
		unsigned int size = result[i].size();
		// remove all previous clique-vertices from i'th clique
		for (unsigned int prv=0; prv<i; ++prv) {
			BKSet diff = setDifference(result[i], result[prv]);
			result[i].swap(diff);
		}
		if (result[i].size() != size) { // ordering may have changed - resort
			auto begin = result.begin(); std::advance(begin, i);
			std::sort(begin, result.end(), [] (const BKSet& a, const BKSet& b) { return a.size() > b.size(); });
		} else {
			++i;
		}
	}

	// remove empty cliques
	auto newEnd = std::remove_if(result.begin(), result.end(), [] (const BKSet& clique ) { return clique.size() == 0; });
	result.resize(std::distance(result.begin(), newEnd));

	return result;
}

void bronKerbosch1(BKSet R, BKSet P, BKSet X, const BKNeigh& N, BKResult& result) { // see wikipedia [Bron-Kerbosch algorithm] for this
	if (!P.size() && !X.size()) {
		result.push_back(R);
		return;
	}
	for (auto v : P) {
		R.insert(v);
		bronKerbosch1(R, setIntersection(P, N[v]), setIntersection(X, N[v]), N, result);
		P.erase(v);
		X.insert(v);
	}
}

template <class T>
inline std::set<T> setUnion(const std::set<T>& a, const std::set<T>& b) {
	std::set<T> result;
	std::set_union(a.begin(), a.end(), b.begin(), b.end(), std::inserter(result, result.end()));
	return result;
}

template <class T>
inline std::set<T> setIntersection(const std::set<T>& a, const std::set<T>& b) {
	std::set<T> result;
	std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), std::inserter(result, result.end()));
	return result;
}

template <class T>
inline std::set<T> setDifference(const std::set<T>& a, const std::set<T>& b) {
	std::set<T> result;
	std::set_difference(a.begin(), a.end(), b.begin(), b.end(), std::inserter(result, result.end()));
	return result;
}
