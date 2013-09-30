#ifndef DECOMPOSE_H_
#define DECOMPOSE_H_

#include <vector>
using std::vector;

#include <memory>

#include <functional>

#include "Iterate.h"
#include "Traverse.h"

#ifdef USE_LAPACKPP
#include <lapackpp/gmd.h>
#include <lapackpp/laslv.h>
#endif

namespace Graph {
namespace Algorithm {

template <class Graph>
struct Decompose {
	typedef typename Graph::GraphPtr  GraphPtr;
	typedef typename Graph::Node      Node;
	typedef typename Graph::NodePtr   NodePtr;
	typedef typename Graph::NodeList  NodeList;
	typedef typename Graph::Edge      Edge;
	typedef typename Graph::EdgePtr   EdgePtr;
	typedef typename Graph::EdgeList  EdgeList;

	static vector<GraphPtr> connectedComponents(Graph& graph) {
		vector<GraphPtr> result;
		NodeList& nodeList = graph.getNodes();
		if (!nodeList.size()) return result;
		// "unvisit" all nodes and edges first
		Iterate<Graph>::iterateNodes( graph, std::bind(&Node::setVisited, std::_Placeholder<1>(), false) );
		Iterate<Graph>::iterateEdges( graph, std::bind(&Edge::setVisited, std::_Placeholder<1>(), false) );

		bool nodesLeft = true;
		typename NodeList::iterator nextNode = nodeList.begin();
		while (nodesLeft) {
			GraphPtr newGraph(new Graph());
			Traverse<Graph>::depthFirstTraverse(
				// node visitor
				[&newGraph] (NodePtr n) {
					NodePtr x = newGraph->getNodeForId(n->getId());
					if (!x) {
						x = newGraph->addNode();
						Graph::copyNodeAttributes(n, x);
					}
				},
				/* edge callback */
				[&newGraph] (EdgePtr edge) {
					NodePtr source = edge->getSourceNode();
					NodePtr target = edge->getTargetNode();
					NodePtr x = newGraph->getNodeForId(source->getId());
					NodePtr y = newGraph->getNodeForId(target->getId());
					if (!x) { x = newGraph->addNode(); Graph::copyNodeAttributes(source, x); }
					if (!y) { y = newGraph->addNode(); Graph::copyNodeAttributes(target, y); }
					EdgePtr e = x->linkTo(y);
					if (e) Graph::copyEdgeAttributes(edge,e);
					edge->setVisited(true);
				},
			*nextNode);
			newGraph->copyAttributes(graph);

			// Copy remaining edges.
			EdgeList& edges = graph.getEdges();
			for (typename EdgeList::iterator eIt = edges.begin(); eIt != edges.end(); ++eIt) {
				if ((*eIt)->isVisited()) continue;
				if (!(*eIt)->getSourceNode()->isVisited() && !(*eIt)->getTargetNode()->isVisited()) continue;
				NodePtr n0 = newGraph->getNodeForId((*eIt)->getSourceNode()->getId());
				NodePtr n1 = newGraph->getNodeForId((*eIt)->getTargetNode()->getId());
				EdgePtr e = n0->linkTo(n1);
				Graph::copyEdgeAttributes(*eIt, e);
				(*eIt)->setVisited(true);
			}

			result.push_back(newGraph);

			// determine if there are nodes left
			nodesLeft = false;
			for (typename NodeList::iterator it = nextNode; it != nodeList.end(); ++it) {
				if (!(*it)->isVisited()) {
					nodesLeft = true;
					nextNode = it;
					break;
				}
			}
		}

		// unvisit all nodes
		Iterate<Graph>::iterateNodes( graph, std::bind(&Node::setVisited, std::_Placeholder<1>(), false) );
		return result;
	}

#ifdef USE_LAPACKPP
	class TreeNode {
		public:
			TreeNode() : subGraph(), childLeft(0), childRight(0) {}
			~TreeNode() { if (childLeft) delete childLeft; if (childRight) delete childRight; }
			GraphPtr subGraph;
			TreeNode* childLeft;
			TreeNode* childRight;
	};
	typedef std::shared_ptr<TreeNode>      SpectralDecomposition;

	static SpectralDecomposition spectralDecomposition(GraphPtr graph, unsigned int minSize) {
		SpectralDecomposition result(new TreeNode());
		result->subGraph = graph;
		(*graph)["sdLevel"] = 0;
		spectralDecomposition(result.get(), minSize);

		return result;
	}

	protected:
		static void spectralDecomposition(TreeNode* node, unsigned int minSize) {
			if (!node || node->subGraph->getNodeCount() <= minSize) return;

			LaGenMatDouble lMatrix = node->subGraph->getLaplacianMatrix<LaGenMatDouble>(true);

			LaVectorDouble eigValueR(lMatrix.rows()), eigValueI(lMatrix.rows());
			LaGenMatDouble eigVectors;
			LaEigSolve(lMatrix, eigValueR, eigValueI, eigVectors);

			GraphPtr rootGraph = node->subGraph;
			Iterate<Graph>::iterateEdges( *rootGraph, std::bind(&Edge::setVisited, std::_Placeholder<1>(), false) );

			GraphPtr left(new Graph());
			GraphPtr right(new Graph());
			int level = any_cast<int>((*rootGraph)["sdLevel"]);
			(*left)["sdLevel"] = level + 1;
			(*right)["sdLevel"] = level + 1;
			Iterate<Graph>::iterateNodes(*(node->subGraph), std::bind(&spectralDivide, left, right, eigVectors, std::_Placeholder<1>()));
			Iterate<Graph>::iterateNodes(*(node->subGraph), std::bind(&spectralLink, left, right, eigVectors, std::_Placeholder<1>()));

			if (!left->getNodeCount() || !right->getNodeCount()) return;

			TreeNode* child = new TreeNode();
			child->subGraph = left;
			node->childLeft = child;

			child = new TreeNode();
			child->subGraph = right;
			node->childRight = child;

			spectralDecomposition(node->childLeft, minSize);
			spectralDecomposition(node->childRight, minSize);
		}

		static void spectralDivide(GraphPtr left, GraphPtr right, LaGenMatDouble& eigVectors, NodePtr node) {
			NodePtr newNode;
			newNode = (eigVectors(node->getId(), 1) < 0) ? left->addNode() : right->addNode();
			Graph::copyNodeAttributes(node, newNode);
		}
		static void spectralLink(GraphPtr left, GraphPtr right, LaGenMatDouble& eigVectors, NodePtr node) {
			bool isLeft = (eigVectors(node->getId(), 1) < 0);
			NodePtr self = isLeft ? left->getNodeForId(node->getId()) : right->getNodeForId(node->getId());
			if (!self) return;

			list<EdgePtr> outgoing = node->getEdges();
			for (typename list<EdgePtr>::iterator eIt = outgoing.begin(); eIt != outgoing.end(); ++eIt) {
				NodePtr otherOld = (*eIt)->getSourceNode()->getId() == node->getId() ? (*eIt)->getTargetNode() : (*eIt)->getSourceNode();
				NodePtr other = isLeft ? left->getNodeForId(otherOld->getId()) : right->getNodeForId(otherOld->getId());
				// is edge a split edge? if it is ignore this one
				if (!other || (*eIt)->isVisited()) continue;
				(*eIt)->setVisited(true);
				// ..else create new edge and copy attributes
				EdgePtr newEdge = self->linkTo(other);
				Graph::copyEdgeAttributes(*eIt, newEdge);
			}
		}
#endif
};

} // Algorithm
} // Graph

#endif /* DECOMPOSE_H_ */
