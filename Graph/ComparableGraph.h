#ifndef COMPARABLEGRAPH_H
#define COMPARABLEGRAPH_H

#include <memory>
using std::shared_ptr;
#include <map>
#include <functional>
#include <stdexcept>
#include <utility>

#include <boost/optional.hpp>

#include "UndirectedGraph.h"

namespace Graph {

template <typename NodeDescType, typename EdgeDescType>
class ComparableGraph : public UndirectedGraph {
	public:
		typedef ComparableGraph<NodeDescType, EdgeDescType> Graph;
		typedef shared_ptr<Graph> GraphPtr;

	protected:
		// Data storage typedefs
		typedef std::map<UId, NodeDescType> NodeDescMap;
		typedef std::map<UId, EdgeDescType> EdgeDescMap;

		typedef std::map<UId, float> NodeInsertRemoveCostMap;
		typedef std::map<UId, float> EdgeInsertRemoveCostMap;

	public:
		ComparableGraph() : UndirectedGraph() {}
		virtual ~ComparableGraph() {}

		// Descriptor getters/setters
		const NodeDescType& getNodeDesc(ConstNodePtr node) const;
		const EdgeDescType& getEdgeDesc(ConstEdgePtr edge) const;

		void setNodeDesc(ConstNodePtr node, const NodeDescType& desc);
		void setEdgeDesc(ConstEdgePtr edge, const EdgeDescType& desc);

		// Cost getters
		float getNodeSubstituteCost(const ComparableGraph& g0, ConstNodePtr n0, const ComparableGraph& g1, ConstNodePtr n1, float c, float l, boost::optional<float> precomputedDistance = boost::none) const;
		float getEdgeSubstituteCost(const ComparableGraph& g0, ConstEdgePtr e0, const ComparableGraph& g1, ConstEdgePtr e1, float c, float l) const;

		float getNodeInsertRemoveCost(ConstNodePtr n, float c, float l) const;
		float getEdgeInsertRemoveCost(ConstEdgePtr e, float c, float l) const;

		// Serialization
		virtual void serializeGraphMLToString(string& content, const Indentation& indent = Indentation());
		virtual void deserializeGraphMLFromString(const std::string graphMLString);

		// Misc functions
		static void copyNodeAttributes(NodePtr from, NodePtr to);
		static void combineNodeAttributes(NodePtr from, NodePtr to);
		static void copyEdgeAttributes(EdgePtr from, EdgePtr to);
		static void combineEdgeAttributes(EdgePtr from, EdgePtr to);
		//void flushCostCaches();

	protected:
		// Descriptor storage
		NodeDescMap m_nodeDescs;
		EdgeDescMap m_edgeDescs;

		// Insert/remove cost storage (cache)
		//NodeInsertRemoveCostMap m_nodeInsertRemoveCosts;
		//EdgeInsertRemoveCostMap m_edgeInsertRemoveCosts;
};

// Descriptor getters/setters

template <typename NodeDescType, typename EdgeDescType>
const NodeDescType& ComparableGraph<NodeDescType, EdgeDescType>::getNodeDesc(ConstNodePtr node) const {
	// Try to fetch descriptor for given node.
	typename NodeDescMap::const_iterator it = m_nodeDescs.find(node->getId());
	// If there is no descriptor for this node, there is not much we can do.
	if (it == m_nodeDescs.end()) throw std::runtime_error("No descriptor for node.");
	return it->second;
}

template <typename NodeDescType, typename EdgeDescType>
const EdgeDescType& ComparableGraph<NodeDescType, EdgeDescType>::getEdgeDesc(ConstEdgePtr edge) const {
	// Try to fetch descriptor for given edge.
	typename EdgeDescMap::const_iterator it = m_edgeDescs.find(edge->getId());
	// If there is no descriptor for this edge, there is not much we can do.
	if (it == m_edgeDescs.end()) throw std::runtime_error("No descriptor for edge.");
	return it->second;
}

template <typename NodeDescType, typename EdgeDescType>
void ComparableGraph<NodeDescType, EdgeDescType>::setNodeDesc(ConstNodePtr node, const NodeDescType& desc) {
	// Using insert here avoids the need for NodeDescType to be default constructible.
	m_nodeDescs.insert(std::make_pair(node->getId(), desc));
}

template <typename NodeDescType, typename EdgeDescType>
void ComparableGraph<NodeDescType, EdgeDescType>::setEdgeDesc(ConstEdgePtr edge, const EdgeDescType& desc) {
	// Using insert here avoids the need for EdgeDescType to be default constructible.
	m_edgeDescs.insert(std::make_pair(edge->getId(), desc));
}

// Cost getters

template <typename NodeDescType, typename EdgeDescType>
float ComparableGraph<NodeDescType, EdgeDescType>::getNodeSubstituteCost(const ComparableGraph& g0, ConstNodePtr n0, const ComparableGraph& g1, ConstNodePtr n1, float c, float l, boost::optional<float> precomputedDistance) const {
	return NodeDescType::nodeSubstituteCost(g0, n0, g1, n1, c, l, precomputedDistance);
}

template <typename NodeDescType, typename EdgeDescType>
float ComparableGraph<NodeDescType, EdgeDescType>::getEdgeSubstituteCost(const ComparableGraph& g0, ConstEdgePtr e0, const ComparableGraph& g1, ConstEdgePtr e1, float c, float l) const {
	return EdgeDescType::edgeSubstituteCost(g0, e0, g1, e1, c, l);
}

template <typename NodeDescType, typename EdgeDescType>
float ComparableGraph<NodeDescType, EdgeDescType>::getNodeInsertRemoveCost(ConstNodePtr n, float c, float l) const {
	// Try to fetch insert/remove costs for given node from cache.
	//typename NodeInsertRemoveCostMap::iterator it = m_nodeInsertRemoveCosts.find(n->getId());
	// If there is no cost in the cache, compute and store it.
	//if (it == m_nodeInsertRemoveCosts.end()) {
		float cost = NodeDescType::nodeInsertRemoveCost(*this, n, c, l);
	//	m_nodeInsertRemoveCosts[n->getId()] = cost;
		return cost;
	//}
	// Otherwise, return cached cost.
	//return it->second;
}

template <typename NodeDescType, typename EdgeDescType>
float ComparableGraph<NodeDescType, EdgeDescType>::getEdgeInsertRemoveCost(ConstEdgePtr e, float c, float l) const {
	// Try to fetch insert/remove costs for given edge from cache.
	//typename EdgeInsertRemoveCostMap::iterator it = m_edgeInsertRemoveCosts.find(e->getId());
	// If there is no cost in the cache, compute and store it.
	//if (it == m_edgeInsertRemoveCosts.end()) {
		float cost = EdgeDescType::edgeInsertRemoveCost(*this, e, c, l);
	//	m_edgeInsertRemoveCosts[e->getId()] = cost;
		return cost;
	//}
	// Otherwise, return cached cost.
	//return it->second;
}

template <typename NodeDescType, typename EdgeDescType>
void ComparableGraph<NodeDescType, EdgeDescType>::serializeGraphMLToString(string& content, const Indentation& indent) {
	NodeList& nodeList = getNodes();
	for (NodeList::iterator nIt = nodeList.begin(); nIt != nodeList.end(); ++nIt) {
		(**nIt)["NodeDesc"] = getNodeDesc(*nIt).serialize();
	}

	EdgeList& edgeList = getEdges();
	for (EdgeList::iterator eIt = edgeList.begin(); eIt != edgeList.end(); ++eIt) {
		(**eIt)["EdgeDesc"] = getEdgeDesc(*eIt).serialize();
	}

	return UndirectedGraph::serializeGraphMLToString(content, indent);
}

template <typename NodeDescType, typename EdgeDescType>
void ComparableGraph<NodeDescType, EdgeDescType>::deserializeGraphMLFromString(const std::string graphMLString) {
	UndirectedGraph::deserializeGraphMLFromString(graphMLString);

	//flushCostCaches();
	m_nodeDescs.clear();
	m_edgeDescs.clear();

	NodeList& nodeList = getNodes();
	for (NodeList::iterator nIt = nodeList.begin(); nIt != nodeList.end(); ++nIt) {
		NodeDescType nodeDesc;
		nodeDesc.deserialize(any_cast<string>((**nIt)["NodeDesc"]));
		m_nodeDescs.insert(std::make_pair((*nIt)->getId(), nodeDesc));
	}

	EdgeList& edgeList = getEdges();
	for (EdgeList::iterator eIt = edgeList.begin(); eIt != edgeList.end(); ++eIt) {
		EdgeDescType edgeDesc;
		edgeDesc.deserialize(any_cast<string>((**eIt)["EdgeDesc"]));
		m_edgeDescs.insert(std::make_pair((*eIt)->getId(), edgeDesc));
	}
}

template <typename NodeDescType, typename EdgeDescType>
void ComparableGraph<NodeDescType, EdgeDescType>::copyNodeAttributes(NodePtr from, NodePtr to) {
	UndirectedGraph::copyNodeAttributes(from, to);
	combineNodeAttributes(from, to);
}

template <typename NodeDescType, typename EdgeDescType>
void ComparableGraph<NodeDescType, EdgeDescType>::combineNodeAttributes(NodePtr from, NodePtr to) {
	UndirectedGraph::combineNodeAttributes(from, to);
	dynamic_cast<ComparableGraph<NodeDescType, EdgeDescType>*>(to->getGraphPtr())->setNodeDesc(to, dynamic_cast<ComparableGraph<NodeDescType, EdgeDescType>*>(from->getGraphPtr())->getNodeDesc(from));
}

template <typename NodeDescType, typename EdgeDescType>
void ComparableGraph<NodeDescType, EdgeDescType>::copyEdgeAttributes(EdgePtr from, EdgePtr to) {
	UndirectedGraph::copyEdgeAttributes(from, to);
	combineEdgeAttributes(from, to);
}

template <typename NodeDescType, typename EdgeDescType>
void ComparableGraph<NodeDescType, EdgeDescType>::combineEdgeAttributes(EdgePtr from, EdgePtr to) {
	UndirectedGraph::combineEdgeAttributes(from, to);
	dynamic_cast<ComparableGraph<NodeDescType, EdgeDescType>*>(to->getGraphPtr())->setEdgeDesc(to, dynamic_cast<ComparableGraph<NodeDescType, EdgeDescType>*>(from->getGraphPtr())->getEdgeDesc(from));
}

/*
template <typename NodeDescType, typename EdgeDescType>
void ComparableGraph<NodeDescType, EdgeDescType>::flushCostCaches() {
	m_nodeInsertRemoveCosts.clear();
	m_edgeInsertRemoveCosts.clear();
}
*/

}

#endif // COMPARABLEGRAPH_H
