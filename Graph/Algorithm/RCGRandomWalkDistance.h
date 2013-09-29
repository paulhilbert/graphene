#ifndef RCGRANDOMWALKDISTANCE_H_
#define RCGRANDOMWALKDISTANCE_H_

#include <cstring>
#include <limits>
#include <boost/any.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
using boost::optional;
#include <boost/none.hpp>
using boost::none;
#include <boost/tuple/tuple.hpp>
using boost::tuple;
#include <iomanip>
#include <functional>
using namespace std::placeholders;
#include <limits>

#include <Math/Distances.h>
#include <Graph/RCGFast.h>
#include <Graph/Algorithm.h>
#include <Optimization/Munkres.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
using namespace Eigen;

#include <cmath>

namespace Graph {
namespace Algorithm {

// RCGRandomWalkCostsConfig declaration

struct RCGRandomWalkCostsConfig {
	RCGRandomWalkCostsConfig() :
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
};

// RCGRandomWalkDistance declaration

class RCGRandomWalkDistance {
public:
	RCGRandomWalkDistance();
	virtual ~RCGRandomWalkDistance();
	void discardNodeMemory();
	void discardEdgeMemory();
	void reserveNodeMemory(unsigned int nodeCount);
	void reserveEdgeMemory(unsigned int edgeCount);
	tuple<float, float, float, float> distGraphs(RCGFast& g0, RCGFast& g1, const RCGRandomWalkCostsConfig& editCosts, optional<float> sup = none);
	static float edgeInsertRemoveCosts(RCGFast::EdgePtr edge, RCGFast& g);

protected:
	static const int TYPE_ROOM = 0;
	static const int TYPE_OUTSIDE = 1;
	static const int TYPE_BALCONY = 2;
	static const int TYPE_DOOR = 0;
	static const int TYPE_WINDOW = 1;
	static const int TYPE_STAIRS = 4;

protected:
	float distNodes(RCGFast::NodePtr n0, RCGFast::NodePtr n1, RCGFast& g0, RCGFast& g1, const RCGRandomWalkCostsConfig& editCosts, bool colorize = false);
	float compNodes(RCGFast::NodePtr n0, RCGFast::NodePtr n1, RCGFast& g0, RCGFast& g1, const RCGRandomWalkCostsConfig& editCosts);
	float compEdges(RCGFast::EdgePtr e0, RCGFast::EdgePtr e1, RCGFast::NodePtr sourceNode0, RCGFast::NodePtr sourceNode1, RCGFast& g0, RCGFast& g1, const RCGRandomWalkCostsConfig& editCosts);

	void computeEdgeInsertRemoveCosts(RCGFast& g);
	float gerschgorinMaxSup(const MatrixXf& m);

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

// RCGRandomWalkDistance implementation

RCGRandomWalkDistance::RCGRandomWalkDistance() :
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

RCGRandomWalkDistance::~RCGRandomWalkDistance() {
	discardNodeMemory();
	discardEdgeMemory();
}

inline void RCGRandomWalkDistance::discardNodeMemory() {
	if (m_nodeCostArray) delete m_nodeCostArray; m_nodeCostArray = NULL;
	if (m_nodeMunkres)   delete m_nodeMunkres;   m_nodeMunkres   = NULL;
	if (m_nodeRowMate)   delete[] m_nodeRowMate; m_nodeRowMate   = NULL;
	if (m_nodeColMate)   delete[] m_nodeColMate; m_nodeColMate   = NULL;
	m_maxNodeCount = 0;
}

inline void RCGRandomWalkDistance::discardEdgeMemory() {
	if (m_edgeCostArray) delete m_edgeCostArray; m_edgeCostArray = NULL;
	if (m_edgeMunkres)   delete m_edgeMunkres;   m_edgeMunkres   = NULL;
	if (m_edgeRowMate)   delete[] m_edgeRowMate; m_edgeRowMate   = NULL;
	if (m_edgeColMate)   delete[] m_edgeColMate; m_edgeColMate   = NULL;
	m_maxNodeDegree = 0;
}

inline void RCGRandomWalkDistance::reserveNodeMemory(unsigned int nodeCount) {
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

inline void RCGRandomWalkDistance::reserveEdgeMemory(unsigned int edgeCount) {
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

tuple<float, float, float, float> RCGRandomWalkDistance::distGraphs(RCGFast& g0, RCGFast& g1, const RCGRandomWalkCostsConfig& editCosts, optional<float> sup) {
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

	assert(n);
	assert(m);

	if (!size) return 0.f;

	RCGFast::NodeList& n0 = g0.getNodes();
	RCGFast::NodeList& n1 = g1.getNodes();

	RCGFast::EdgeList& e0 = g0.getEdges();
	RCGFast::EdgeList& e1 = g1.getEdges();

	map<RCGFast::UId, int> n0IdMap;
	map<RCGFast::UId, int> n1IdMap;

	int nIdx = 0;
	for (auto nIt = n0.begin(); nIt != n0.end(); ++nIt, ++nIdx) {
		n0IdMap[(*nIt)->getId()] = nIdx;
	}
	nIdx = 0;
	for (auto nIt = n1.begin(); nIt != n1.end(); ++nIt, ++nIdx) {
		n1IdMap[(*nIt)->getId()] = nIdx;
	}

	MatrixXf adj = MatrixXf::Zero(n*m, n*m);
	for (auto e0It = e0.begin(); e0It != e0.end(); ++e0It) {
		for (auto e1It = e1.begin(); e1It != e1.end(); ++e1It) {
			RCGFast::NodePtr e0s = (*e0It)->getSourceNode();
			RCGFast::NodePtr e0t = (*e0It)->getTargetNode();
			RCGFast::NodePtr e1s = (*e1It)->getSourceNode();
			RCGFast::NodePtr e1t = (*e1It)->getTargetNode();

			int a = n0IdMap[e0s->getId()] * m + n1IdMap[e1s->getId()];
			int b = n0IdMap[e0t->getId()] * m + n1IdMap[e1t->getId()];
			int c = n0IdMap[e0s->getId()] * m + n1IdMap[e1t->getId()];
			int d = n0IdMap[e0t->getId()] * m + n1IdMap[e1s->getId()];

			assert(a >= 0 && a < static_cast<int>(n*m));
			assert(b >= 0 && b < static_cast<int>(n*m));
			assert(c >= 0 && c < static_cast<int>(n*m));
			assert(d >= 0 && d < static_cast<int>(n*m));

			float sigma = 1;
			float kn0 = exp(-distNodes(e0s, e1s, g0, g1, editCosts) / sigma);
			float kn1 = exp(-distNodes(e0t, e1t, g0, g1, editCosts) / sigma);

			adj(a, b) = kn0 * kn1;
			adj(b, a) = kn0 * kn1;

			kn0 = exp(-distNodes(e0s, e1t, g0, g1, editCosts) / sigma);
			kn1 = exp(-distNodes(e0t, e1s, g0, g1, editCosts) / sigma);

			adj(c, d) = kn0 * kn1;
			adj(d, c) = kn0 * kn1;
		}
	}


	if (adj.isZero()) {
		return boost::make_tuple(0.f, 0.f, 0.f, 0.f);
	}

	// If sup is not set at all, just return the computed approximation.
	if (!sup) {
		return boost::make_tuple(gerschgorinMaxSup(adj), 0.f, 0.f, 0.f);
	}

	// If sup is set but is zero, directly use a newly computed approximation.
	if (!sup.get()) {
		sup = gerschgorinMaxSup(adj);
	}

	if (sup.get() == 0.f) {
		cout << "SUP IS ZERO" << endl;
		getchar();
	}



	// Inversion version
	const float c = 1.5f;
	float lambda = 1.f / (c * sup.get());

	FullPivLU<MatrixXf> lu( MatrixXf::Identity(m*n, m*n) - lambda * adj );

	if (!lu.isInvertible()) {
		cout << "NOT INVERTIBLE!" << endl;
		getchar();
		return 0.f;
	}

	// Four versions: With(out) normalization, with(out) "- n*m"
	float result = lu.inverse().sum();
	return boost::make_tuple(result, result - n*m, result / (n*m*n*m), (result - n*m) / (n*m*n*m));
	// END inversion version



	//vector<MatrixXf> matrices;

	auto A = adj;

	int k = 2;
	for (int i = 0; i < k-1; ++i) {
		adj *= adj;
		A   += adj;
		//matrices.push_back(A);
	}

	float sum = A.sum();

	if (isinf(sum)) {
		cout << "SUM IS INF" << endl;
		/*
		for (uint i = 0; i < matrices.size(); ++i) {
			ofstream out("/home/ochi/Documents/Studium/tmp/" + lexical_cast<string>(i));
			out << matrices[i];
			out.close();
		}
		*/
		getchar();
	}

	if (isnan(sum)) {
		cout << "SUM IS NAN" << endl;
		getchar();
	}

	return sum;

	//~ cout << "Computing " << any_cast<string>(g0["ObjId"]) + "-" + any_cast<string>(g1["ObjId"]) << endl << std::flush;

	//~ EigenSolver<MatrixXf> eig;
	//~ eig.compute(adj);
	//~ auto D = eig.eigenvalues();

	//~ ofstream out("/home/ochi/Documents/Studium/tmp/" + any_cast<string>(g0["ObjId"]) + "-" + any_cast<string>(g1["ObjId"]));
	//~ out << adj;
	//~ out.close();

	//~ ofstream out1("/home/ochi/Documents/Studium/tmp/" + any_cast<string>(g0["ObjId"]) + "-" + any_cast<string>(g1["ObjId"]) + "-evals");
	//~ out1 << D;
	//~ out1.close();

	//~ getchar();

	//~ if (eig.eigenvalues().rows() == 0) {
		//~ cout << "Outputting" << endl;

	//~ }

	//if (isnan(D.maxCoeff())) {
		//~ ofstream out("/home/ochi/Documents/Studium/tmp/" + any_cast<string>(g0["ObjId"]) + "-" + any_cast<string>(g1["ObjId"]));
		//~ out << adj << std::flush;
		//~ out.close();

		//~ ofstream out1("/home/ochi/Documents/Studium/tmp/" + any_cast<string>(g0["ObjId"]) + "-" + any_cast<string>(g1["ObjId"]) + "-eigs");
		//~ out1 << D << std::flush;
		//~ out1.close();

		//getchar();
	//}

	//~ cout << "a: " << adj.rows() << ", " << adj.cols() << endl;
	//~ cout << "E: " << eig.eigenvectors().rows() << ", " << eig.eigenvectors().cols() << endl;

	//~ cout << "D: " << D.rows() << ", " << D.cols() << endl;

	//~ for (uint i = 0; i < 6; ++i) {
		//~ D = D.array() * D.array();
	//~ }

	//~ cout << "D: " << D.rows() << ", " << D.cols() << endl;

	//cout << D.maxCoeff() << ", " << D.minCoeff() << endl;

	//~ auto A = (eig.eigenvectors() * D.asDiagonal() * eig.eigenvectors().adjoint()).real();

	//~ cout << "A: " << A.rows() << ", " << A.cols() << endl;

	//cout << A.rows() << ", " << A.cols() << endl;

	//~ if (isinf(A.maxCoeff())) {
		//~ cout << "Inf" << endl;
	//~ }

	//~ return A.sum();

	//~ auto A = adj;
	//~ //MatrixXf A = MatrixXf::Identity(m*n, m*n);
	//~ for (uint i = 0; i < 6; ++i) {
		//~ adj *= adj;
		//~ A   += adj;
	//~ }
	//~ return A.sum();
	//return (adj*adj*adj*adj*adj*adj*adj*adj).sum();

	/*
	FullPivLU<MatrixXf> lu( MatrixXf::Identity(m*n, m*n) - lambda * adj );

	if (!lu.isInvertible()) {
		cout << "NOT INVERTIBLE!" << endl;
		return 0.f;
	}

	return (lu.inverse().sum());// / (n*m*n*m);// / (4.f * g0.getEdgeCount() * g1.getEdgeCount());  // / (n*m*n*m);
	*/
}

////////////////////////////////////////
////////////////////////////////////////
//             distNodes              //
////////////////////////////////////////
////////////////////////////////////////

float RCGRandomWalkDistance::distNodes(RCGFast::NodePtr n0, RCGFast::NodePtr n1, RCGFast& g0, RCGFast& g1, const RCGRandomWalkCostsConfig& editCosts, bool colorize) {
	RCGFast::EdgeList e0 = n0->getNonVisitedEdges();
	RCGFast::EdgeList e1 = n1->getNonVisitedEdges();

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

inline float RCGRandomWalkDistance::compNodes(RCGFast::NodePtr n0, RCGFast::NodePtr n1, RCGFast& g0, RCGFast& g1, const RCGRandomWalkCostsConfig& editCosts) {
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

inline float RCGRandomWalkDistance::compEdges(RCGFast::EdgePtr e0, RCGFast::EdgePtr e1, RCGFast::NodePtr sourceNode0, RCGFast::NodePtr sourceNode1, RCGFast& g0, RCGFast& g1, const RCGRandomWalkCostsConfig& editCosts) {
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

float RCGRandomWalkDistance::edgeInsertRemoveCosts(RCGFast::EdgePtr edge, RCGFast& g) {
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

void RCGRandomWalkDistance::computeEdgeInsertRemoveCosts(RCGFast& g) {
	RCGFast::EdgeList& edges = g.getEdges();
	for (RCGFast::EdgeList::iterator eIt = edges.begin(); eIt != edges.end(); ++eIt) {
		float costs = edgeInsertRemoveCosts(*eIt, g);
		//(**eIt)["InsertRemoveCosts"] = costs;
		g.m_attribsPassageInsertRemoveCosts[(*eIt)->getId()] = costs;
	}
}

float RCGRandomWalkDistance::gerschgorinMaxSup(const MatrixXf& m) {
	float sup = 0.f;
	for (int j = 0; j < m.cols(); ++j) {
		float sum = 0.f;
		for (int i = 0; i < m.rows(); ++i) {
			if (i == j) continue;
			sum += fabs(m(i, j));
		}
		float t = m(j, j) > 0.f ? m(j, j) + sum : fabs(m(j, j) - sum);
		if (t > sup) sup = t;
	}
	return sup;
}

} // Algorithm
} // Graph

#endif // RCGRANDOMWALKDISTANCE_H_
