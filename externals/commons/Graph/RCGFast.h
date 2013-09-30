#ifndef RCGFAST_H_
#define RCGFAST_H_

#include <functional>

#include <boost/graph/floyd_warshall_shortest.hpp>

#include "RCG.h"

namespace Graph {

class RCGFast : public RCG {
public:
	typedef std::shared_ptr<RCGFast> GraphPtr;
public:
	RCGFast() : RCG() { }
	virtual ~RCGFast() {}

	vector<int>             m_attribsRoomType;
	vector<float>           m_attribsRoomArea;
	vector<int>             m_attribsPassageType;
	vector<float>           m_attribsPassageInsertRemoveCosts;
	vector< vector<float> > m_attribsNodeDesc;

	void precomputeAttribs() {
		m_attribsRoomType.clear();
		m_attribsRoomArea.clear();
		m_attribsPassageType.clear();
		m_attribsPassageInsertRemoveCosts.clear();
		m_attribsNodeDesc.clear();

		for (NodeList::iterator nIt = m_nodes.begin(); nIt != m_nodes.end(); ++nIt) {
			UId id = (*nIt)->getId();
			if (m_attribsRoomType.size() < id+1) {
				m_attribsRoomType.resize(id+1, -1);
				m_attribsRoomArea.resize(id+1, 0.f);
				m_attribsNodeDesc.resize(id+1);
			}

			try {
				m_attribsRoomType[id] = any_cast<int>((**nIt)["RoomType"]);
			} catch (...) {}

			try {
				m_attribsRoomArea[id] = any_cast<float>((**nIt)["RoomArea"]);
			} catch (...) {}

			try {
				m_attribsNodeDesc[id] = any_cast< vector<float> >((**nIt)["NodeDesc"]);
			} catch (...) {}
		}

		for (EdgeList::iterator eIt = m_edges.begin(); eIt != m_edges.end(); ++eIt) {
			UId id = (*eIt)->getId();
			if (m_attribsPassageType.size() < id+1) {
				m_attribsPassageType.resize(id+1, -1);
				m_attribsPassageInsertRemoveCosts.resize(id+1, 0.f);
			}

			try {
				m_attribsPassageType[id] = any_cast<int>((**eIt)["PassageType"]);
			} catch (...) {}

			try {
				m_attribsPassageInsertRemoveCosts[id] = any_cast<float>((**eIt)["InsertRemoveCosts"]);
			} catch (...) {}
		}
	}
};

}

namespace boost {
	template <>
	struct graph_traits<Graph::RCGFast> {
		typedef Graph::RCGFast::NodePtr vertex_descriptor;
		typedef Graph::RCGFast::EdgePtr edge_descriptor;
		typedef Graph::RCGFast::NodeIter vertex_iterator;
		typedef Graph::RCGFast::EdgeIter edge_iterator;

		typedef directed_tag directed_category;
		typedef allow_parallel_edge_tag edge_parallel_category;

		typedef int vertices_size_type;
		typedef int edges_size_type;
	};
	// VertexListGraph
	inline std::pair<typename graph_traits<Graph::RCGFast>::vertex_iterator, typename graph_traits<Graph::RCGFast>::vertex_iterator>
	vertices(Graph::RCGFast& g) {
		Graph::RCGFast::NodeList& nodes = g.getNodes();
		return make_pair(nodes.begin(), nodes.end());
	}
	inline graph_traits<Graph::RCGFast>::vertices_size_type num_vertices(const Graph::RCGFast& g) {
		return g.getNodeCount();
	}
	// EdgeListGraph
	inline std::pair<typename graph_traits<Graph::RCGFast>::edge_iterator, typename graph_traits<Graph::RCGFast>::edge_iterator>
	edges(Graph::RCGFast& g) {
		Graph::RCGFast::EdgeList& nodes = g.getEdges();
		return make_pair(nodes.begin(), nodes.end());
	}
	inline graph_traits<Graph::RCGFast>::edges_size_type num_edges(const Graph::RCGFast& g) {
		return g.getEdgeCount();
	}
	inline graph_traits<Graph::RCGFast>::vertex_descriptor
	source(graph_traits<Graph::RCGFast>::edge_descriptor e, const Graph::RCGFast& g) {
		return e->getSourceNode();
	}
	inline graph_traits<Graph::RCGFast>::vertex_descriptor
	target(graph_traits<Graph::RCGFast>::edge_descriptor e, const Graph::RCGFast& g) {
		return e->getTargetNode();
	}
}

#endif /* RCGFAST_H_ */
