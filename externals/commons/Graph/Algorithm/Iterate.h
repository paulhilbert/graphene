#ifndef ITERATE_H_
#define ITERATE_H_

#include <functional>

namespace Graph {
namespace Algorithm {

template <class Graph>
struct Iterate {
	typedef typename Graph::NodePtr   NodePtr;
	typedef typename Graph::NodeList  NodeList;
	typedef typename Graph::EdgePtr   EdgePtr;
	typedef typename Graph::EdgeList  EdgeList;

	static void iterateNodes(Graph& graph, std::function<void (NodePtr)> nodeVisitor) {
		NodeList& nodeList = graph.getNodes();
		std::for_each(nodeList.begin(), nodeList.end(), nodeVisitor);
	}
	static void iterateEdges(Graph& graph, std::function<void (EdgePtr)> edgeVisitor, std::function<bool (EdgePtr)> edgeAcceptor = NULL) {
		EdgeList& edgeList = graph.getEdges();
		for (typename EdgeList::iterator it = edgeList.begin(); it != edgeList.end(); ++it) {
			if (edgeAcceptor ? edgeAcceptor(*it) : true) {
				edgeVisitor(*it);
			}
		}
	}
	static void iterate1Ring(NodePtr node, std::function<void (EdgePtr)> edgeVisitor, std::function<bool (EdgePtr)> edgeAcceptor = NULL) {
		EdgeList& edgeList = node->getEdges();
		for (typename EdgeList::iterator it = edgeList.begin(); it != edgeList.end(); ++it) {
			if (edgeAcceptor ? edgeAcceptor(*it) : true) {
				edgeVisitor(*it);
			}
		}
	}
};

} // Algorithm
} // Graph

#endif /* ITERATE_H_ */
