#ifndef EDITDISTANCE_H_
#define EDITDISTANCE_H_

#include <cmath>
#include <boost/optional.hpp>
#include <Optimization/Munkres.h>

#ifdef USE_CUDA
#include "../../GPU/CublasDist.h"
using namespace GPU;
#endif

namespace Graph {
namespace Algorithm {

struct EditDistConf {
	float nodeSubstC;
	float nodeSubstL;
	float nodeInsRemC;
	float nodeInsRemL;
	float edgeSubstC;
	float edgeSubstL;
	float edgeInsRemC;
	float edgeInsRemL;
	float nodeVsEdgeL;
};

template<typename GraphType>
class EditDistance {
	public:
		typedef typename GraphType::ConstNodePtr ConstNodePtr;
		typedef typename GraphType::ConstEdgePtr ConstEdgePtr;
		typedef GraphType Point;
		typedef float     Scalar;
	public:
		EditDistance(EditDistConf conf);
		virtual ~EditDistance();

		// MMM (Munkres Memory Management)
		void discardNodeMemory();
		void discardEdgeMemory();
		void reserveNodeMemory(unsigned int nodeCount);
		void reserveEdgeMemory(unsigned int edgeCount);

		// The ultimate graph distance computation function.
#ifdef USE_CUDA
		float distGraphs(const GraphType& g0, const GraphType& g1, const vector<vector<float>>& cudaDist, unsigned int cudaOff);
#endif
		float distGraphs(const GraphType& g0, const GraphType& g1);

	protected:
		// Secondary Munkres for node distance computation.
		float distNodes(const GraphType& g0, ConstNodePtr n0, const GraphType& g1, ConstNodePtr n1, optional<float> nodeDist = none);

		// Munkres computation and storage stuff
		::Data::RawArray2D<float>* m_nodeCostArray;
		::Data::RawArray2D<float>* m_edgeCostArray;
		Optimization::Munkres<float>* m_nodeMunkres;
		Optimization::Munkres<float>* m_edgeMunkres;
		long* m_nodeRowMate;
		long* m_nodeColMate;
		long* m_edgeRowMate;
		long* m_edgeColMate;

		// For remembering how much memory is currently reserved.
		unsigned int m_amountNodeMemoryReserved;
		unsigned int m_amountEdgeMemoryReserved;

		// This factor determines how much more memory than
		// currently necessary is going to be reserved (for
		// future computations).
		static const int MEMORY_RESERVATION_FACTOR = 2;

		// Configuration
		EditDistConf m_conf;
};

/////////////////////
// Implementations //
/////////////////////

// RCGEditDistance implementation

template<typename GraphType>
EditDistance<GraphType>::EditDistance(EditDistConf conf) :
		m_nodeCostArray(NULL),
		m_edgeCostArray(NULL),
		m_nodeMunkres(NULL),
		m_edgeMunkres(NULL),
		m_nodeRowMate(NULL),
		m_nodeColMate(NULL),
		m_edgeRowMate(NULL),
		m_edgeColMate(NULL),
		m_amountNodeMemoryReserved(0),
		m_amountEdgeMemoryReserved(0),
		m_conf(conf) {
}

template<typename GraphType>
EditDistance<GraphType>::~EditDistance() {
	discardNodeMemory();
	discardEdgeMemory();
}

template<typename GraphType>
void EditDistance<GraphType>::discardNodeMemory() {
	if (m_nodeCostArray) delete m_nodeCostArray; m_nodeCostArray = NULL;
	if (m_nodeMunkres)   delete m_nodeMunkres;   m_nodeMunkres   = NULL;
	if (m_nodeRowMate)   delete[] m_nodeRowMate; m_nodeRowMate   = NULL;
	if (m_nodeColMate)   delete[] m_nodeColMate; m_nodeColMate   = NULL;
	m_amountNodeMemoryReserved = 0;
}

template<typename GraphType>
void EditDistance<GraphType>::discardEdgeMemory() {
	if (m_edgeCostArray) delete m_edgeCostArray; m_edgeCostArray = NULL;
	if (m_edgeMunkres)   delete m_edgeMunkres;   m_edgeMunkres   = NULL;
	if (m_edgeRowMate)   delete[] m_edgeRowMate; m_edgeRowMate   = NULL;
	if (m_edgeColMate)   delete[] m_edgeColMate; m_edgeColMate   = NULL;
	m_amountEdgeMemoryReserved = 0;
}

template<typename GraphType>
void EditDistance<GraphType>::reserveNodeMemory(unsigned int nodeCount) {
	if (nodeCount > m_amountNodeMemoryReserved) {
		discardNodeMemory();
		m_nodeCostArray = new ::Data::RawArray2D<float>(MEMORY_RESERVATION_FACTOR*nodeCount, MEMORY_RESERVATION_FACTOR*nodeCount, 0.f);
		m_nodeMunkres = new Optimization::Munkres<float>(MEMORY_RESERVATION_FACTOR*nodeCount);
		m_nodeRowMate = new long[MEMORY_RESERVATION_FACTOR*nodeCount];
		m_nodeColMate = new long[MEMORY_RESERVATION_FACTOR*nodeCount];
		m_amountNodeMemoryReserved = MEMORY_RESERVATION_FACTOR*nodeCount;
	}
}

template<typename GraphType>
void EditDistance<GraphType>::reserveEdgeMemory(unsigned int edgeCount) {
	if (edgeCount > m_amountEdgeMemoryReserved) {
		discardEdgeMemory();
		m_edgeCostArray = new ::Data::RawArray2D<float>(MEMORY_RESERVATION_FACTOR*edgeCount, MEMORY_RESERVATION_FACTOR*edgeCount, 0.f);
		m_edgeMunkres = new Optimization::Munkres<float>(MEMORY_RESERVATION_FACTOR*edgeCount);
		m_edgeRowMate = new long[MEMORY_RESERVATION_FACTOR*edgeCount];
		m_edgeColMate = new long[MEMORY_RESERVATION_FACTOR*edgeCount];
		m_amountEdgeMemoryReserved = MEMORY_RESERVATION_FACTOR*edgeCount;
	}
}

////////////////////////////////////////
////////////////////////////////////////
//             distGraphs             //
////////////////////////////////////////
////////////////////////////////////////

#ifdef USE_CUDA
template<typename GraphType>
float EditDistance<GraphType>::distGraphs(const GraphType& g0, const GraphType& g1, const vector<vector<float>>& cudaDist, unsigned int cudaOff) {
	unsigned int n = g0.getNodeCount();
	unsigned int m = g1.getNodeCount();
	unsigned int size = n + m;

	// If there are no nodes at all, just return zero.
	if (!size) return 0.f;

	reserveNodeMemory(size);

	m_nodeCostArray->setRange(0,    n, m, size, std::numeric_limits<float>::max()); // Upper right
	m_nodeCostArray->setRange(n, size, 0,    m, std::numeric_limits<float>::max()); // Lower left
	m_nodeCostArray->setRange(n, size, m, size, 0.f); // Lower right

	// Node substitutions (upper left entries)
	typename GraphType::NodeList::const_iterator nIt = g0.getNodes().begin();
	for (unsigned int i = 0; i < n; ++i, ++nIt) {
		unsigned int di = boost::any_cast<unsigned int>((**nIt)["cudaPos"]);
		typename GraphType::NodeList::const_iterator mIt = g1.getNodes().begin();
		for (unsigned int j = 0; j < m; ++j, ++mIt) {
			unsigned int dj = cudaOff + j;
			float nDist = cudaDist[di][dj];
			(*m_nodeCostArray)(i, j) = distNodes(g0, *nIt, g1, *mIt, (nDist < 0.f) ? 0.f : sqrtf(nDist)); // Cap because nDist seems to become numerically negative sometimes.
		}
	}

	// Node removals (upper right entries)
	nIt = g0.getNodes().begin();
	for (unsigned int i=0; i < n; ++i, ++nIt) {
		(*m_nodeCostArray)(i, m+i) = g0.getNodeInsertRemoveCost(*nIt, m_conf.nodeInsRemC, m_conf.nodeInsRemL);
	}

	// Node insertions (lower left entries)
	nIt = g1.getNodes().begin();
	for (unsigned int i=0; i < m; ++i, ++nIt) {
		(*m_nodeCostArray)(i+n, i) = g1.getNodeInsertRemoveCost(*nIt, m_conf.nodeInsRemC, m_conf.nodeInsRemL);
	}

	// Do the Munkres.
	m_nodeMunkres->solve(*m_nodeCostArray, size, m_nodeColMate, m_nodeRowMate);

	// Sum up all the costs.
	float costs = 0.f;
	for (unsigned int i=0; i < size; ++i) {
		costs += (*m_nodeCostArray)(i, m_nodeColMate[i]);
	}

	// Return costs normalized by total node count.
	return costs / static_cast<float>(size);
}
#endif

template<typename GraphType>
float EditDistance<GraphType>::distGraphs(const GraphType& g0, const GraphType& g1) {
	unsigned int n = g0.getNodeCount();
	unsigned int m = g1.getNodeCount();
	unsigned int size = n + m;

	// If there are no nodes at all, just return zero.
	if (!size) return 0.f;

	reserveNodeMemory(size);

	m_nodeCostArray->setRange(0,    n, m, size, std::numeric_limits<float>::max()); // Upper right
	m_nodeCostArray->setRange(n, size, 0,    m, std::numeric_limits<float>::max()); // Lower left
	m_nodeCostArray->setRange(n, size, m, size, 0.f); // Lower right

	// Node substitutions (upper left entries)
	typename GraphType::NodeList::const_iterator nIt = g0.getNodes().begin();
	for (unsigned int i = 0; i < n; ++i, ++nIt) {
		typename GraphType::NodeList::const_iterator mIt = g1.getNodes().begin();
		for (unsigned int j = 0; j < m; ++j, ++mIt) {
			(*m_nodeCostArray)(i, j) = distNodes(g0, *nIt, g1, *mIt);
		}
	}

	// Node removals (upper right entries)
	nIt = g0.getNodes().begin();
	for (unsigned int i=0; i < n; ++i, ++nIt) {
		(*m_nodeCostArray)(i, m+i) = g0.getNodeInsertRemoveCost(*nIt, m_conf.nodeInsRemC, m_conf.nodeInsRemL);
	}

	// Node insertions (lower left entries)
	nIt = g1.getNodes().begin();
	for (unsigned int i=0; i < m; ++i, ++nIt) {
		(*m_nodeCostArray)(i+n, i) = g1.getNodeInsertRemoveCost(*nIt, m_conf.nodeInsRemC, m_conf.nodeInsRemL);
	}

	// Do the Munkres.
	m_nodeMunkres->solve(*m_nodeCostArray, size, m_nodeColMate, m_nodeRowMate);

	// Sum up all the costs.
	float costs = 0.f;
	for (unsigned int i=0; i < size; ++i) {
		costs += (*m_nodeCostArray)(i, m_nodeColMate[i]);
	}

	// Return costs normalized by total node count.
	return costs / static_cast<float>(size);
}

////////////////////////////////////////
////////////////////////////////////////
//             distNodes              //
////////////////////////////////////////
////////////////////////////////////////

template<typename GraphType>
float EditDistance<GraphType>::distNodes(const GraphType& g0, ConstNodePtr n0, const GraphType& g1, ConstNodePtr n1, optional<float> nodeDist) {
	unsigned int n = n0->getEdgeCount();
	unsigned int m = n1->getEdgeCount();
	unsigned int size = n + m;

	// Compute the basic node substitution costs first.
	float nodeCosts = m_conf.nodeVsEdgeL * g0.getNodeSubstituteCost(g0, n0, g1, n1, m_conf.nodeSubstC, m_conf.nodeSubstL, nodeDist);

	// If there are no edges at all, just return the substitution costs.
	if (!size) return nodeCosts;

	// Optimization: If node lambda is 1 (=> edge lambda is 0), just return costs.
	if (m_conf.nodeVsEdgeL == 1.f) {
		return nodeCosts;
	}

	reserveEdgeMemory(size);

	//Data::RawArray2D<float> C(size, size);
	m_edgeCostArray->setRange(0,    n, m, size, std::numeric_limits<float>::max()); // Upper right
	m_edgeCostArray->setRange(n, size, 0,    m, std::numeric_limits<float>::max()); // Lower left
	m_edgeCostArray->setRange(n, size, m, size, 0.f); // Lower right

	// Edge substitutions (upper left entries)
	typename GraphType::EdgeList::const_iterator e0It = n0->getEdges().begin();
	for (unsigned int i = 0; i < n; ++i, ++e0It) {
		typename GraphType::EdgeList::const_iterator e1It = n1->getEdges().begin();
		for (unsigned int j = 0; j < m; ++j, ++e1It) {
			(*m_edgeCostArray)(i, j) = g0.getEdgeSubstituteCost(g0, *e0It, g1, *e1It, m_conf.edgeSubstC, m_conf.edgeSubstL);
		}
	}

	// Edge removals (upper right entries)
	e0It = n0->getEdges().begin();
	for (unsigned int i=0; i < n; ++i, ++e0It) {
		(*m_edgeCostArray)(i, m+i) = g0.getEdgeInsertRemoveCost(*e0It, m_conf.edgeInsRemC, m_conf.edgeInsRemL);
	}

	// Edge insertions (lower left entries)
	e0It = n1->getEdges().begin();
	for (unsigned int i=0; i < m; ++i, ++e0It) {
		(*m_edgeCostArray)(i+n, i) = g1.getEdgeInsertRemoveCost(*e0It, m_conf.edgeInsRemC, m_conf.edgeInsRemL);
	}

	// Do the Munkres.
	m_edgeMunkres->solve(*m_edgeCostArray, size, m_edgeColMate, m_edgeRowMate);

	// Add edge costs to the total costs.
	float edgeCosts = 0.f;
	for (unsigned int i=0; i < size; ++i) {
		edgeCosts += (*m_edgeCostArray)(i, m_edgeColMate[i]);
	}

	// Return the costs normalized by total edge count.
	return nodeCosts + (1.f - m_conf.nodeVsEdgeL) * (edgeCosts / static_cast<float>(size));
}

} // Algorithm
} // Graph

#endif // EDITDISTANCE_H_
