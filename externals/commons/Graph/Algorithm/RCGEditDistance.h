#ifndef RCGEDITDISTANCE_H_
#define RCGEDITDISTANCE_H_

#include <cstring>
#include <limits>
#include <boost/any.hpp>
#include <boost/lexical_cast.hpp>
#include <iomanip>
#include <functional>
using namespace std::placeholders;

#include <Math/Distances.h>
#include <Graph/RCGFast.h>
#include <Graph/Algorithm.h>
#include <Optimization/Munkres.h>

namespace Graph {
namespace Algorithm {

// RCGEditCostsConfig declaration

struct RCGEditCostsConfig {
	RCGEditCostsConfig() :
		insertRoom(0.5f),
		removeRoom(1.f),
		insertPassage(0.25f),
		removePassage(0.5f),
		areaPenaltyMultiplier(0.2f),
		correctRoomType(0.f),
		wrongRoomType(1000000.f),
		correctPassageType(0.f),
		wrongPassageType(1000000.f),
		matchEdgeLambda(0.25f),
		insertDeleteEdgeLambda(.6f),
		secondPassLambda(1.5),
		edgeCostSumOffset(0.1f),
		edgeCostSumOffsetClamp(0.75f),
		nodeDescLambda(5.f) {
	}

	float insertRoom;
	float removeRoom;
	float insertPassage;
	float removePassage;
	float areaPenaltyMultiplier;
	float correctRoomType;
	float wrongRoomType;
	float correctPassageType;
	float wrongPassageType;
	float matchEdgeLambda;
	float insertDeleteEdgeLambda;
	float secondPassLambda;
	float edgeCostSumOffset;
	float edgeCostSumOffsetClamp;
	float nodeDescLambda;

	/*
	std::string getAsText() {
		std::string result;
		result += "iR " + boost::lexical_cast<std::string>(insertRoom) + " ";
		result += "rR " + boost::lexical_cast<std::string>(removeRoom) + " ";
		result += "iP " + boost::lexical_cast<std::string>(insertPassage) + " ";
		result += "rP " + boost::lexical_cast<std::string>(removePassage) + " ";
		result += "APM " + boost::lexical_cast<std::string>(areaPenaltyMultiplier) + " ";
		result += "cR " + boost::lexical_cast<std::string>(correctRoomType) + " ";
		result += "wR " + boost::lexical_cast<std::string>(wrongRoomType) + " ";
		result += "cP " + boost::lexical_cast<std::string>(correctPassageType) + " ";
		result += "wP " + boost::lexical_cast<std::string>(wrongPassageType);
		return result;
	}
	*/
};

// RCGEditDistance declaration

class RCGEditDistance {
public:
	RCGEditDistance();
	virtual ~RCGEditDistance();
	void discardNodeMemory();
	void discardEdgeMemory();
	void reserveNodeMemory(unsigned int nodeCount);
	void reserveEdgeMemory(unsigned int edgeCount);
	float distGraphs(RCGFast& g0, RCGFast& g1, const RCGEditCostsConfig& editCosts);
	static float edgeInsertRemoveCosts(RCGFast::EdgePtr edge, RCGFast& g);

protected:
	static const int TYPE_ROOM = 0;
	static const int TYPE_OUTSIDE = 1;
	static const int TYPE_BALCONY = 2;
	static const int TYPE_DOOR = 0;
	static const int TYPE_WINDOW = 1;
	static const int TYPE_STAIRS = 4;

protected:
	float distNodes(RCGFast::NodePtr n0, RCGFast::NodePtr n1, RCGFast& g0, RCGFast& g1, const RCGEditCostsConfig& editCosts, bool colorize = false);
	float compNodes(RCGFast::NodePtr n0, RCGFast::NodePtr n1, RCGFast& g0, RCGFast& g1, const RCGEditCostsConfig& editCosts);
	float compEdges(RCGFast::EdgePtr e0, RCGFast::EdgePtr e1, RCGFast::NodePtr sourceNode0, RCGFast::NodePtr sourceNode1, RCGFast& g0, RCGFast& g1, const RCGEditCostsConfig& editCosts);

	void computeEdgeInsertRemoveCosts(RCGFast& g);

	Data::RawArray2D<float>* m_nodeCostArray;
	Data::RawArray2D<float>* m_edgeCostArray;
	Optimization::Munkres<float>* m_nodeMunkres;
	Optimization::Munkres<float>* m_edgeMunkres;
	long* m_nodeRowMate;
	long* m_nodeColMate;
	long* m_edgeRowMate;
	long* m_edgeColMate;

	unsigned int m_maxNodeCount;
	unsigned int m_maxNodeDegree;
};

/////////////////////
// Implementations //
/////////////////////

// RCGEditDistance implementation

RCGEditDistance::RCGEditDistance() :
	m_nodeCostArray(NULL),
	m_edgeCostArray(NULL),
	m_nodeMunkres(NULL),
	m_edgeMunkres(NULL),
	m_nodeRowMate(NULL),
	m_nodeColMate(NULL),
	m_edgeRowMate(NULL),
	m_edgeColMate(NULL),
	m_maxNodeCount(0),
	m_maxNodeDegree(0) {
}

RCGEditDistance::~RCGEditDistance() {
	discardNodeMemory();
	discardEdgeMemory();
}

inline void RCGEditDistance::discardNodeMemory() {
	if (m_nodeCostArray) delete m_nodeCostArray; m_nodeCostArray = NULL;
	if (m_nodeMunkres)   delete m_nodeMunkres;   m_nodeMunkres   = NULL;
	if (m_nodeRowMate)   delete[] m_nodeRowMate; m_nodeRowMate   = NULL;
	if (m_nodeColMate)   delete[] m_nodeColMate; m_nodeColMate   = NULL;
	m_maxNodeCount = 0;
}

inline void RCGEditDistance::discardEdgeMemory() {
	if (m_edgeCostArray) delete m_edgeCostArray; m_edgeCostArray = NULL;
	if (m_edgeMunkres)   delete m_edgeMunkres;   m_edgeMunkres   = NULL;
	if (m_edgeRowMate)   delete[] m_edgeRowMate; m_edgeRowMate   = NULL;
	if (m_edgeColMate)   delete[] m_edgeColMate; m_edgeColMate   = NULL;
	m_maxNodeDegree = 0;
}

inline void RCGEditDistance::reserveNodeMemory(unsigned int nodeCount) {
	if (nodeCount > m_maxNodeCount) {
		const int FACTOR = 2;
		discardNodeMemory();
		m_nodeCostArray = new Data::RawArray2D<float>(2*FACTOR*nodeCount, 2*FACTOR*nodeCount, 0.f);
		m_nodeMunkres = new Optimization::Munkres<float>(2*FACTOR*nodeCount);
		m_nodeRowMate = new long[2*FACTOR*nodeCount];
		m_nodeColMate = new long[2*FACTOR*nodeCount];
		m_maxNodeCount = FACTOR*nodeCount;
	}
}

inline void RCGEditDistance::reserveEdgeMemory(unsigned int edgeCount) {
	if (edgeCount > m_maxNodeDegree) {
		const int FACTOR = 2;
		discardEdgeMemory();
		m_edgeCostArray = new Data::RawArray2D<float>(2*FACTOR*edgeCount, 2*FACTOR*edgeCount, 0.f);
		m_edgeMunkres = new Optimization::Munkres<float>(2*FACTOR*edgeCount);
		m_edgeRowMate = new long[2*FACTOR*edgeCount];
		m_edgeColMate = new long[2*FACTOR*edgeCount];
		m_maxNodeDegree = FACTOR*edgeCount;
	}
}

////////////////////////////////////////
////////////////////////////////////////
//             distGraphs             //
////////////////////////////////////////
////////////////////////////////////////

float RCGEditDistance::distGraphs(RCGFast& g0, RCGFast& g1, const RCGEditCostsConfig& editCosts) {
	// Unvisit all nodes
	Iterate<RCGFast>::iterateNodes( g0, std::bind(&RCGFast::Node::setVisited, std::_Placeholder<1>(), false) );
	Iterate<RCGFast>::iterateNodes( g1, std::bind(&RCGFast::Node::setVisited, std::_Placeholder<1>(), false) );
	Iterate<RCGFast>::iterateEdges( g0, std::bind(&RCGFast::Edge::setVisited, std::_Placeholder<1>(), false) );
	Iterate<RCGFast>::iterateEdges( g1, std::bind(&RCGFast::Edge::setVisited, std::_Placeholder<1>(), false) );

	// Recompute edge insert/remove costs
	computeEdgeInsertRemoveCosts(g0);
	computeEdgeInsertRemoveCosts(g1);

	unsigned int n = g0.getNodeCount();
	unsigned int m = g1.getNodeCount();
	unsigned int size = n + m;

	if (!size) return 0.f;

	reserveNodeMemory(size);

	//Data::RawArray2D<float> C(size, size);
	m_nodeCostArray->setRange(0,    n, m, size, std::numeric_limits<float>::max()); // Upper right
	m_nodeCostArray->setRange(n, size, 0,    m, std::numeric_limits<float>::max()); // Lower left
	m_nodeCostArray->setRange(n, size, m, size, 0.f); // Lower right

	// Node substitutions (upper left entries)
	RCGFast::NodeList::const_iterator nIt = g0.getNodes().begin();
	for (unsigned int i = 0; i < n; ++i, ++nIt) {
		RCGFast::NodeList::const_iterator mIt = g1.getNodes().begin();
		for (unsigned int j = 0; j < m; ++j, ++mIt) {
			(*m_nodeCostArray)(i, j) = distNodes(*nIt, *mIt, g0, g1, editCosts);
		}
	}

	// Node removals (upper right entries)
	nIt = g0.getNodes().begin();
	for (unsigned int i=0; i < n; ++i, ++nIt) {
		RCGFast::EdgeList edges = (*nIt)->getEdges();
		float edgeCostSum = 0.f;
		for (RCGFast::EdgeList::iterator eIt = edges.begin(); eIt != edges.end(); ++eIt) {
			edgeCostSum += editCosts.insertDeleteEdgeLambda * g0.m_attribsPassageInsertRemoveCosts[(*eIt)->getId()];
		}
		float offset = editCosts.edgeCostSumOffset * g0.m_attribsRoomArea[(*nIt)->getId()];
		(*m_nodeCostArray)(i, m+i) = edgeCostSum + (editCosts.edgeCostSumOffsetClamp < offset ? editCosts.edgeCostSumOffsetClamp : offset);
	}

	// Node insertions (lower left entries)
	nIt = g1.getNodes().begin();
	for (unsigned int i=0; i < m; ++i, ++nIt) {
		RCGFast::EdgeList edges = (*nIt)->getEdges();
		float edgeCostSum = 0.f;
		for (RCGFast::EdgeList::iterator eIt = edges.begin(); eIt != edges.end(); ++eIt) {
			edgeCostSum += editCosts.insertDeleteEdgeLambda * g1.m_attribsPassageInsertRemoveCosts[(*eIt)->getId()];
		}
		float offset = editCosts.edgeCostSumOffset * g1.m_attribsRoomArea[(*nIt)->getId()];
		(*m_nodeCostArray)(i+n, i) = edgeCostSum + (editCosts.edgeCostSumOffsetClamp < offset ? editCosts.edgeCostSumOffsetClamp : offset);
	}

	m_nodeMunkres->solve(*m_nodeCostArray, size, m_nodeColMate, m_nodeRowMate);

	RCGFast::NodeList g0Nodes = g0.getNodes();
	RCGFast::NodeList g1Nodes = g1.getNodes();

#ifdef DEBUG_EDIT_DIST
	std::cout << "n = " << n << ", m = " << m << std::endl;

	for (unsigned int y = 0; y < size; ++y) {
		for (unsigned int x = 0; x < size; ++x) {
			std::cout << std::setw(12) << (*m_nodeCostArray)(y, x);
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	for (unsigned int x = 0; x < size; ++x) {
		std::cout << std::setw(12) << m_nodeColMate[x];
	}
	std::cout << std::endl;

	RCGFast::GraphPtr matchGraph(new RCGFast());

	std::map<unsigned int, RCGFast::NodePtr> g0NodeMap;
	std::map<unsigned int, RCGFast::NodePtr> g1NodeMap;

	std::map<RCGFast::NodePtr, RCGFast::NodePtr> g0NodePtrMap;
	std::map<RCGFast::NodePtr, RCGFast::NodePtr> g1NodePtrMap;

	RCGFast::EdgeList g0Edges = g0.getEdges();
	RCGFast::EdgeList g1Edges = g1.getEdges();

	unsigned int nodeIdx = 0;
	for (RCGFast::NodeList::iterator nIt = g0Nodes.begin(); nIt != g0Nodes.end(); ++nIt) {
		RCGFast::NodePtr n = matchGraph->addNode();
		n->combineAttributes(*nIt);
		g0NodeMap[nodeIdx++] = n;
		g0NodePtrMap[*nIt] = n;
	}

	const float g1Delta = 20.f;

	nodeIdx = 0;
	for (RCGFast::NodeList::iterator nIt = g1Nodes.begin(); nIt != g1Nodes.end(); ++nIt) {
		RCGFast::NodePtr n = matchGraph->addNode();
		n->combineAttributes(*nIt);
		try {
			any_cast< Vector<float, 3>& >((*n)["Position"]) += Vector<float, 3>(0.f, 0.f, g1Delta);
			vector< Vector<float, 3> >& vertices = any_cast< vector< Vector<float, 3> >& >((*n)["RoomVertices"]);
			for (unsigned int v = 0; v < vertices.size(); ++v) {
				vertices[v] += Vector<float, 3>(0.f, 0.f, g1Delta);
			}
		} catch (...) {
			std::cout << "Warning: Node in target graph without position or vertices." << std::endl;
		}
		g1NodeMap[nodeIdx++] = n;
		g1NodePtrMap[*nIt] = n;
	}
#endif

	float costs = 0.f;

	for (unsigned int i=0; i < size; ++i) {
		unsigned int j = m_nodeColMate[i];
		costs += (*m_nodeCostArray)(i, j);
	}

	/*

	// Set "removed" nodes and adjacent edges to visited and sum up these costs
	for (unsigned int i=0; i < size; ++i) {
		unsigned int j = m_nodeColMate[i];

		if (i >= n && j < m) { // Node in graph 1 removed
			RCGFast::NodeList::iterator n1It = g1Nodes.begin(); std::advance(n1It, j);
			(*n1It)->setVisited(true);
			Iterate<RCGFast>::iterate1Ring( *n1It, bind(&RCGFast::Edge::setVisited, arg1, true) );
			costs += (*m_nodeCostArray)(i, j);
		} else if (i < n && j >= m) { // Node in graph 0 removed
			RCGFast::NodeList::iterator n0It = g0Nodes.begin(); std::advance(n0It, i);
			(*n0It)->setVisited(true);
			Iterate<RCGFast>::iterate1Ring( *n0It, bind(&RCGFast::Edge::setVisited, arg1, true) );
			costs += (*m_nodeCostArray)(i, j);
		}
	}


	costs *= editCosts.secondPassLambda / editCosts.insertDeleteEdgeLambda;

	// Recompute edge insert/remove costs
	computeEdgeInsertRemoveCosts(g0);
	computeEdgeInsertRemoveCosts(g1);

	//float oldInsertDeleteEdgeLambda = editCosts.insertDeleteEdgeLambda;

	// Add up the remaining costs for the matchings without the removed entities
	for (unsigned int i=0; i < size; ++i) {
		unsigned int j = m_nodeColMate[i];

		if (i < n && j < m) {
			RCGFast::NodeList::iterator n0It = g0Nodes.begin(); std::advance(n0It, i);
			RCGFast::NodeList::iterator n1It = g1Nodes.begin(); std::advance(n1It, j);
			costs += distNodes(*n0It, *n1It, g0, g1, editCosts, true);
			//(*g0NodeMap[i])["Color"] = Vector<float, 3>(0.7, 0.7, 0.0);
			//(*g1NodeMap[j])["Color"] = Vector<float, 3>(0.7, 0.0, 0.7);
		}

#ifdef DEBUG_EDIT_DIST
		if (i < n && j < m) {
			RCGFast::EdgePtr edge = g0NodeMap[i]->linkTo( g1NodeMap[j] );
			RCGFast::NodeList::iterator n0It = g0Nodes.begin(); std::advance(n0It, i);
			RCGFast::NodeList::iterator n1It = g1Nodes.begin(); std::advance(n1It, j);
			//distNodes(*n0It, *n1It, g0, g1, editCosts, true);
			(*edge)["PassageType"] = 4;
			(*edge)["MatchCosts"] = (*m_nodeCostArray)(i, j);
			//(*g0NodeMap[i])["CostMatrixCoord"] = "i = " + boost::lexical_cast<string>(i);
			//(*g1NodeMap[j])["CostMatrixCoord"] = "j = " + boost::lexical_cast<string>(j);
		} else if (i >= n && j < m) {
			(*g1NodeMap[j])["Color"] = Vector<float, 3>(1.f, 0.f, 0.f);
			//(*g1NodeMap[j])["CostMatrixCoord"] = "j = " + boost::lexical_cast<string>(j);
		} else if (i < n && j >= m) {
			(*g0NodeMap[i])["Color"] = Vector<float, 3>(1.f, 0.f, 0.f);
			//(*g0NodeMap[i])["CostMatrixCoord"] = "i = " + boost::lexical_cast<string>(i);
		}
#endif
	}

#ifdef DEBUG_EDIT_DIST
	for (RCGFast::EdgeList::iterator eIt = g0Edges.begin(); eIt != g0Edges.end(); ++eIt) {
		RCGFast::NodePtr n0 = (*eIt)->getSourceNode();
		RCGFast::NodePtr n1 = (*eIt)->getTargetNode();
		RCGFast::EdgePtr edge = g0NodePtrMap[n0]->linkTo(g0NodePtrMap[n1]);
		RCGFast::combineEdgeAttributes(*eIt, edge);
		(*eIt)->removeAttribute("EdgeColor");
	}

	for (RCGFast::EdgeList::iterator eIt = g1Edges.begin(); eIt != g1Edges.end(); ++eIt) {
		RCGFast::NodePtr n0 = (*eIt)->getSourceNode();
		RCGFast::NodePtr n1 = (*eIt)->getTargetNode();
		RCGFast::EdgePtr edge = g1NodePtrMap[n0]->linkTo(g1NodePtrMap[n1]);
		RCGFast::combineEdgeAttributes(*eIt, edge);
		try {
			any_cast< Vector<float, 3>& >((*edge)["OpeningPosition"]) += Vector<float, 3>(0.f, 0.f, g1Delta);
			any_cast< float& >((*edge)["OpeningTop"])    += g1Delta;
			any_cast< float& >((*edge)["OpeningBottom"]) += g1Delta;
		} catch (...) {
			std::cout << "Warning: Edge in target graph without passage position or geometry." << std::endl;
		}
		(*eIt)->removeAttribute("EdgeColor");
	}

	matchGraph->serializeGraphML("matchGraph.graphml");
#endif

	*/

	return costs / static_cast<float>(size);
}

////////////////////////////////////////
////////////////////////////////////////
//             distNodes              //
////////////////////////////////////////
////////////////////////////////////////

float RCGEditDistance::distNodes(RCGFast::NodePtr n0, RCGFast::NodePtr n1, RCGFast& g0, RCGFast& g1, const RCGEditCostsConfig& editCosts, bool colorize) {
	RCGFast::EdgeList e0 = n0->getNonVisitedEdges();
	RCGFast::EdgeList e1 = n1->getNonVisitedEdges();

	/*
	if (colorize) {
		for (RCGFast::EdgeList::iterator it = e0.begin(); it != e0.end(); ++it)
			(**it)["EdgeColor"] = Vector<float, 3>(0.7f, 0.7f, 0.0f);
		for (RCGFast::EdgeList::iterator it = e1.begin(); it != e1.end(); ++it)
			(**it)["EdgeColor"] = Vector<float, 3>(0.7f, 0.0f, 0.7f);
	}
	*/

#ifdef DEBUG_EDIT_DIST
	/*
	try {
		const float debugArea0 = 7.90994;
		const float debugArea1 = 7.90994;
		const float debugEps = 0.005;
		if (!colorize &&
		    (((fabs(any_cast<float>((*n0)["RoomArea"]) - debugArea0) < debugEps) && (fabs(any_cast<float>((*n1)["RoomArea"]) - debugArea1) < debugEps)) ||
		    ((fabs(any_cast<float>((*n1)["RoomArea"]) - debugArea0) < debugEps) && (fabs(any_cast<float>((*n0)["RoomArea"]) - debugArea1) < debugEps)))) {
			std::cout << "Breakpoint" << std::endl;
		}
	} catch (...) {}
	*/
#endif

	float costs = 0.f;

#ifndef EDITDIST_NO_TOPOLOGY
	unsigned int n = e0.size();
	unsigned int m = e1.size();
	unsigned int size = n + m;

	if (size) {
		reserveEdgeMemory(size);

		//Data::RawArray2D<float> C(size, size);
		m_edgeCostArray->setRange(0,    n, m, size, std::numeric_limits<float>::max()); // Upper right
		m_edgeCostArray->setRange(n, size, 0,    m, std::numeric_limits<float>::max()); // Lower left
		m_edgeCostArray->setRange(n, size, m, size, 0.f); // Lower right

		// Edge substitutions (upper left entries)
		RCGFast::EdgeList::const_iterator e0It = e0.begin();
		for (unsigned int i = 0; i < n; ++i, ++e0It) {
			RCGFast::EdgeList::const_iterator e1It = e1.begin();
			for (unsigned int j = 0; j < m; ++j, ++e1It) {
				(*m_edgeCostArray)(i, j) = compEdges(*e0It, *e1It, n0, n1, g0, g1, editCosts);
			}
		}

		// Edge removals (upper right entries)
		RCGFast::EdgeList::const_iterator eIt = e0.begin();
		for (unsigned int i=0; i < n; ++i, ++eIt) {
			(*m_edgeCostArray)(i, m+i) = editCosts.insertDeleteEdgeLambda * g0.m_attribsPassageInsertRemoveCosts[(*eIt)->getId()];
		}

		// Edge insertions (lower left entries)
		eIt = e1.begin();
		for (unsigned int i=0; i < m; ++i, ++eIt) {
			(*m_edgeCostArray)(i+n, i) = editCosts.insertDeleteEdgeLambda * g1.m_attribsPassageInsertRemoveCosts[(*eIt)->getId()];
		}

		m_edgeMunkres->solve(*m_edgeCostArray, size, m_edgeColMate, m_edgeRowMate);

		for (unsigned int i=0; i < size; ++i) {
			unsigned int j = m_edgeColMate[i];
			costs += (*m_edgeCostArray)(i, j);

	#ifdef DEBUG_EDIT_DIST
			if (colorize) {
				if (i >= n && j < m) {
					RCGFast::EdgeList::iterator it = e1.begin(); std::advance(it, j);
					(**it)["EdgeColor"] = Vector<float,3>(1.f, static_cast<float>((*it)->get< Vector<float, 3> >("EdgeColor") == none), 0.f);
				} else if (i < n && j >= m) {
					RCGFast::EdgeList::iterator it = e0.begin(); std::advance(it, i);
					(**it)["EdgeColor"] = Vector<float,3>(1.f, static_cast<float>((*it)->get< Vector<float, 3> >("EdgeColor") == none), 0.f);
				}
			}
	#endif
		}
	}
#endif

	return compNodes(n0, n1, g0, g1, editCosts) + costs;
}

inline float RCGEditDistance::compNodes(RCGFast::NodePtr n0, RCGFast::NodePtr n1, RCGFast& g0, RCGFast& g1, const RCGEditCostsConfig& editCosts) {
#ifdef EDITDIST_NO_ATTRIBS
	return 1.f;
#endif

	int n0Type = g0.m_attribsRoomType[n0->getId()];
	int n1Type = g1.m_attribsRoomType[n1->getId()];

	if (n0Type != n1Type) return 1000000.f;
	if (n0Type == TYPE_OUTSIDE) return 0.f;

	vector<float>& n0Desc = g0.m_attribsNodeDesc[n0->getId()];
	vector<float>& n1Desc = g1.m_attribsNodeDesc[n1->getId()];

	return editCosts.nodeDescLambda * Math::Distances::chiSquare(n0Desc, n1Desc);
/*
	float n0Area = g0.m_attribsRoomArea[n0->getId()];
	float n1Area = g1.m_attribsRoomArea[n1->getId()];

	return editCosts.nodeDescLambda * (1.f - (n0Area < n1Area ? n0Area : n1Area) / (n0Area > n1Area ? n0Area : n1Area));
*/
}

inline float RCGEditDistance::compEdges(RCGFast::EdgePtr e0, RCGFast::EdgePtr e1, RCGFast::NodePtr sourceNode0, RCGFast::NodePtr sourceNode1, RCGFast& g0, RCGFast& g1, const RCGEditCostsConfig& editCosts) {
	int e0Type = g0.m_attribsPassageType[e0->getId()];
	int e1Type = g1.m_attribsPassageType[e1->getId()];

	if (e0Type != e1Type) return 1000000.f;

	// Comment-in for one-ring
	//return 0;

	RCGFast::NodePtr targetNode0 = (e0->getSourceNode() == sourceNode0) ? e0->getTargetNode() : e0->getSourceNode();
	RCGFast::NodePtr targetNode1 = (e1->getSourceNode() == sourceNode1) ? e1->getTargetNode() : e1->getSourceNode();

	float edgeCount0 = static_cast<float>(targetNode0->getNonVisitedEdgeCount());
	float edgeCount1 = static_cast<float>(targetNode1->getNonVisitedEdgeCount());

	float edgeCountMin = (edgeCount0 < edgeCount1 ? edgeCount0 : edgeCount1);
	float edgeCountMax = (edgeCount0 > edgeCount1 ? edgeCount0 : edgeCount1);

	assert(edgeCount0);
	assert(edgeCount1);

	return editCosts.matchEdgeLambda * compNodes(targetNode0, targetNode1, g0, g1, editCosts) + (1.f - editCosts.matchEdgeLambda) * (1.f - edgeCountMin/edgeCountMax);
}

float RCGEditDistance::edgeInsertRemoveCosts(RCGFast::EdgePtr edge, RCGFast& g) {
	if (edge->isVisited()) return 0.f; // This value should be irrelevant.

	RCGFast::NodePtr sourceNode = edge->getSourceNode();
	RCGFast::NodePtr targetNode = edge->getTargetNode();

	int sourceNodeType = g.m_attribsRoomType[sourceNode->getId()];//any_cast<int>((*sourceNode)["RoomType"]);
	int targetNodeType = g.m_attribsRoomType[targetNode->getId()];//any_cast<int>((*targetNode)["RoomType"]);

	float sourceNodeCosts;
	float targetNodeCosts;

	if (targetNodeType == TYPE_OUTSIDE) {
		sourceNodeCosts = 1.f;
	} else {
		float targetArea = g.m_attribsRoomArea[targetNode->getId()];//any_cast<float>((*targetNode)["RoomArea"]);
		float sourceNeighborsArea = 0.f;
		std::set<RCGFast::NodePtr> sourceNeighbors = sourceNode->getNonVisitedNeighbors();
		for (std::set<RCGFast::NodePtr>::const_iterator nIt = sourceNeighbors.begin(); nIt != sourceNeighbors.end(); ++nIt) {
			try {
				sourceNeighborsArea += g.m_attribsRoomArea[(*nIt)->getId()];//any_cast<float>((**nIt)["RoomArea"]);
			} catch (...) {}
		}
		sourceNodeCosts = targetArea / (sourceNeighborsArea > 0.f ? sourceNeighborsArea : 1.f);
	}

	if (sourceNodeType == TYPE_OUTSIDE) {
		targetNodeCosts = 1.f;
	} else {
		float sourceArea = g.m_attribsRoomArea[sourceNode->getId()];//any_cast<float>((*sourceNode)["RoomArea"]);
		float targetNeighborsArea = 0.f;
		std::set<RCGFast::NodePtr> targetNeighbors = targetNode->getNonVisitedNeighbors();
		for (std::set<RCGFast::NodePtr>::const_iterator nIt = targetNeighbors.begin(); nIt != targetNeighbors.end(); ++nIt) {
			try {
				targetNeighborsArea += g.m_attribsRoomArea[(*nIt)->getId()];//any_cast<float>((**nIt)["RoomArea"]);
			} catch (...) {}
		}
		targetNodeCosts = sourceArea / (targetNeighborsArea > 0.f ? targetNeighborsArea : 1.f);
	}

	return 0.5f * (sourceNodeCosts + targetNodeCosts);
}

void RCGEditDistance::computeEdgeInsertRemoveCosts(RCGFast& g) {
	RCGFast::EdgeList& edges = g.getEdges();
	for (RCGFast::EdgeList::iterator eIt = edges.begin(); eIt != edges.end(); ++eIt) {
		float costs = edgeInsertRemoveCosts(*eIt, g);
		//(**eIt)["InsertRemoveCosts"] = costs;
		g.m_attribsPassageInsertRemoveCosts[(*eIt)->getId()] = costs;
	}
}

} // Algorithm
} // Graph

#endif // RCGEDITDISTANCE_H_
