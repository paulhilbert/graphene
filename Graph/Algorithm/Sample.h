#ifndef SAMPLE_H_
#define SAMPLE_H_

#include <vector>
using std::vector;

#include <functional>

#include <Random/RNG.h>
using namespace Random;

#include "Iterate.h"
#include "Decompose.h"
#include "Traverse.h"

namespace Graph {
namespace Algorithm {

template <class Graph>
struct Sample {
	typedef enum { EQUAL, GREATER_OR_EQUAL } SampleSizePolicy;
	typedef enum { RANDOM_WALK, FOREST_FIRE} SampleMethod;

	typedef typename Graph::GraphPtr  GraphPtr;
	typedef typename Graph::Node      Node;
	typedef typename Graph::NodePtr   NodePtr;
	typedef typename Graph::NodeList  NodeList;
	typedef typename Graph::Edge      Edge;
	typedef typename Graph::EdgePtr   EdgePtr;
	typedef typename Graph::EdgeList  EdgeList;
	typedef std::function<bool (EdgePtr edge)>  EdgeAcceptor;


	static NodePtr           randomNode(Graph& graph);
	static GraphPtr          mergeMultiEdges(Graph& graph);
	static vector<GraphPtr>  sampleSubgraphs(Graph& graph, unsigned int count, unsigned int size, SampleMethod method, SampleSizePolicy sizePolicy, EdgeAcceptor edgeAcceptor);
	static GraphPtr          kRingNeighborhood(Graph& graph, NodePtr node, unsigned int k);
};


#include "Sample.inl"

} // Algorithm
} // Graph


#endif /* SAMPLE_H_ */
