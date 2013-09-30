#ifndef CLUSTERGREEDY_H_
#define CLUSTERGREEDY_H_

namespace Cluster {

template <class EntityT, class ClusterT>
using IncPred = std::function<bool (const EntityT&, const ClusterT&)>;
template <class EntityT, class ClusterT>
using Insertion = std::function<void (const EntityT&, ClusterT&)>;
template <class EntityT, class ClusterT>
using Creation = std::function<ClusterT (const EntityT&)>;

template <class EntityT, class ClusterT>
static std::vector<ClusterT> greedy(const std::vector<EntityT>& entities, IncPred<EntityT,ClusterT> incPred, Insertion<EntityT,ClusterT> insert, Creation<EntityT,ClusterT> create);

#include "Greedy.inl"

} // Cluster

#endif /* CLUSTERGREEDY_H_ */
