template <class EntityT, class ClusterT>
inline std::vector<ClusterT> greedy(const std::vector<EntityT>& entities, IncPred<EntityT,ClusterT> incPred, Insertion<EntityT,ClusterT> insert, Creation<EntityT,ClusterT> create) {
	std::vector<ClusterT> result;
	for (const EntityT& entity : entities) {
		// check existing clusters for "near-enough" centers
		bool foundCluster = false;
		for (unsigned int c=0; c<result.size(); ++c) {
			if (!incPred(entity, result[c])) continue;
			// we found a suitable cluster - update center
			insert(entity, result[c]);
			foundCluster = true;
			break;
		}
		if (foundCluster) continue;
		// no cluster found, create new one
		result.push_back(create(entity));
	}
	return result;
}
