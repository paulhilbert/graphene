#ifndef BRONKERBOSH_H_
#define BRONKERBOSH_H_

#include <set>
#include <vector>
#include <algorithm>

// this is a disjoint version of the Bron-Kerbosch algorithm
// (see wikipedia [Bron-Kerbosch algorithm])
// The Bron-Kerbosch algorithm computes all maximal cliques in
// a graph. This algorithm also makes all cliques disjoint
// such that clique maximality is preserved

namespace Cluster {

typedef std::set<int>              BKSet;
typedef std::vector<std::set<int>> BKNeigh;
typedef std::vector<std::set<int>> BKResult;

template <class HasEdge>
static BKResult bronKerbosch(const BKSet& G, const HasEdge& hasEdge);

static void bronKerbosch1(BKSet R, BKSet P, BKSet X, const BKNeigh& N, BKResult& result);

template <class T>
std::set<T> setUnion(const std::set<T>& a, const std::set<T>& b);

template <class T>
std::set<T> setIntersection(const std::set<T>& a, const std::set<T>& b);

template <class T>
std::set<T> setDifference(const std::set<T>& a, const std::set<T>& b);

#include "BronKerbosch.inl"

} // Cluster

#endif /* BRONKERBOSH_H_ */
