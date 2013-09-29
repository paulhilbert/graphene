#ifndef TRAVERSE_H_
#define TRAVERSE_H_

#include <functional>
using namespace std::placeholders;

#include <tuple>
using std::tie;
using std::ignore;

#include <Random/RNG.h>
using namespace Random;


namespace Graph {
namespace Algorithm {

template <class Graph>
class TraverseQueue;

template <class Graph>
struct Traverse {
	// types
	typedef typename Graph::NodePtr   NodePtr;
	typedef typename Graph::NodeList  NodeList;
	typedef typename Graph::EdgePtr   EdgePtr;
	typedef typename Graph::EdgeList  EdgeList;
	typedef typename TraverseQueue<Graph>::QueueElement QueueElement;
	typedef typename RNG::Traits<unsigned int>::GenGeom RNGGeom;

	// functors
	typedef std::function<void (NodePtr)>                         NodeVisitor;
	typedef std::function<void (EdgePtr)>                         EdgeVisitor;
	typedef std::function<bool (NodePtr)>                         NodeAcceptor;
	typedef std::function<bool (EdgePtr)>                         EdgeAcceptor;
	typedef std::function<QueueElement (void)>                    QueueGet;
	typedef std::function<void (NodePtr, EdgePtr, unsigned int)>  QueueInsert;
	typedef std::function<bool (unsigned int nVisited, unsigned int depth, unsigned int maxDepth)>  ContinuePredicate;

	// constants
	static const unsigned int infDepth = std::numeric_limits<unsigned int>::max();

	// functions
	static void traverse(NodeVisitor nodeVisitor, EdgeVisitor edgeVisitor, NodePtr start, QueueGet getNext, QueueInsert putNext, unsigned int maxDepth = infDepth);

	static void   depthFirstTraverse(NodeVisitor nodeVisitor, EdgeVisitor edgeVisitor, NodePtr start, unsigned int maxDepth = infDepth);
	static void breadthFirstTraverse(NodeVisitor nodeVisitor, EdgeVisitor edgeVisitor, NodePtr start, unsigned int maxDepth = infDepth);

	static bool randomWalk(Graph& graph, NodeVisitor nodeVisitor, EdgeVisitor edgeVisitor, NodePtr start, ContinuePredicate contPredicate,	EdgeAcceptor edgeAcceptor);
	static bool forestFire(Graph& graph, NodeVisitor nodeVisitor, EdgeVisitor edgeVisitor, NodePtr start, float fwBurnProb, ContinuePredicate contPredicate, EdgeAcceptor edgeAcceptor);

protected:
	static void traverse(NodeVisitor nodeVisitor, EdgeVisitor edgeVisitor, NodePtr start, QueueGet getNext, QueueInsert putNext, unsigned int depth, unsigned int maxDepth);
	static bool randomWalk(Graph& graph, NodeVisitor nodeVisitor, EdgeVisitor edgeVisitor, NodePtr start,	NodePtr current, ContinuePredicate contPredicate, EdgeAcceptor edgeAcceptor, unsigned int& nodesVisited, unsigned int depth, unsigned int& maxDepth);
	static bool forestFire(Graph& graph, NodeVisitor nodeVisitor, EdgeVisitor edgeVisitor, NodePtr start, NodePtr current, TraverseQueue<Graph>& queue, RNGGeom pRand, ContinuePredicate contPredicate, EdgeAcceptor edgeAcceptor, unsigned int& nodesVisited, unsigned int depth, unsigned int& maxDepth);

};


template <class Graph>
class TraverseQueue {
	protected:
		typedef typename Graph::NodePtr NodePtr;
		typedef typename Graph::EdgePtr EdgePtr;

	public:
		typedef enum { LIFO, FIFO } DequeVariant;
		typedef typename Graph::EdgeList MultiEdge;
		typedef std::tuple<NodePtr,NodePtr,MultiEdge,unsigned int> QueueElement; // (from,to,edges,depth)

	public:
		void          putNext(NodePtr current, EdgePtr e, unsigned int depth, DequeVariant variant);
		QueueElement  getNext(DequeVariant variant);

		unsigned int getQueueSize() const;

	protected:
		deque<QueueElement> m_queue;
};


#include "Traverse.inl"


} // Algorithm
} // Graph


#endif /* TRAVERSE_H_ */
