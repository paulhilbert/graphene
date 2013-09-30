#ifndef UNDIRECTEDGRAPH_H_
#define UNDIRECTEDGRAPH_H_

#include <iostream>
#include <fstream>
#include <list>
#include <set>
#include <vector>
#include <deque>
#include <sstream>
#include <tuple>

using std::list;
using std::vector;
using std::deque;
using std::stringstream;
using std::set;
using std::ofstream;
using std::ifstream;
using std::string;

#include <memory>

#include <boost/optional.hpp>
#include <boost/none.hpp>
using boost::optional;
using boost::none;

#include <functional>
using namespace std::placeholders;

#include <memory>

#include <Generic/RandomAttributes.h>
using namespace Generic;

#include <StringUtils/Indentation.h>
using namespace StringUtils;

#ifdef USE_LAPACKPP
#include <lapackpp/gmd.h>
#include <lapackpp/laslv.h>
#endif


namespace Graph {

class TraverseQueue;

class UndirectedGraph : public RandomAttributes {
	public:
		typedef unsigned int      UId;
		typedef UndirectedGraph   Graph;
		typedef std::shared_ptr<Graph> GraphPtr;

		class Node;
		class Edge;
		typedef Node* NodePtr;
		typedef Node const * ConstNodePtr;
		typedef list<NodePtr> NodeList;
		typedef NodeList::iterator NodeIter;
		typedef Edge* EdgePtr;
		typedef Edge const* ConstEdgePtr;
		typedef list<EdgePtr> EdgeList;
		typedef EdgeList::iterator EdgeIter;

#ifdef USE_LAPACKPP
		typedef LaGenMatDouble LaplacianMatrix;
#endif

	public:
		UndirectedGraph() : m_nextNodeId(0), m_nextEdgeId(0) { }
		virtual ~UndirectedGraph();

		NodePtr addNode();
		NodePtr getFirstNode();

		void removeNode(UId id);
		void removeEdge(UId id);

		NodeList& getNodes();
		EdgeList& getEdges();
		const NodeList& getNodes() const;
		const EdgeList& getEdges() const;

		NodeList getNonVisitedNodes() const;
		EdgeList getNonVisitedEdges() const;

		NodePtr getNodeForId(UId id);
		EdgePtr getEdgeForId(UId id);

		unsigned int getNodeCount() const;
		unsigned int getEdgeCount() const;
		unsigned int getMaxOutdeg() const;

		unsigned int getNonVisitedNodeCount() const;
		unsigned int getNonVisitedEdgeCount() const;

		inline void copyAttributes(Graph& other) {
			m_attributes = other.m_attributes;
			m_nextNodeId = other.m_nextNodeId;
			m_nextEdgeId = other.m_nextEdgeId;
		}

		static void copyNodeAttributes(NodePtr from, NodePtr to);
		static void combineNodeAttributes(NodePtr from, NodePtr to);

		static void copyEdgeAttributes(EdgePtr from, EdgePtr to);
		static void combineEdgeAttributes(EdgePtr from, EdgePtr to);

		void serializeDOT(string name, const char* filename, std::function<string (NodePtr)> getNodeAttributes, std::function<string (EdgePtr)> getEdgeAttributes, const Indentation& indent = Indentation());
		void serializeDOT(string name, ofstream& out, std::function<string (NodePtr)> getNodeAttributes, std::function<string (EdgePtr)> getEdgeAttributes, const Indentation& indent = Indentation());

		virtual void serializeGraphML(string filename, const Indentation& indent = Indentation());
		virtual void serializeGraphMLToString(string& content, const Indentation& indent = Indentation());
		virtual void deserializeGraphML(string filename);
		virtual void deserialize(string filename);

		virtual void deserializeGraphMLFromString(const std::string graphMLString);

#ifdef USE_LAPACKPP
		LaplacianMatrix getLaplacianMatrix(bool normalize = false);
#endif

	protected:
		// management
		void insertEdge(EdgePtr edge);
		void insertNode(NodePtr node);
		void resetNodeIds();
		// (de-)serialization
		enum ParseEntity { GRAPH, NODE, EDGE };
		class ParseInfo {
			public:
				ParseInfo(UndirectedGraph* g) : graph(g), currentEntity(GRAPH) {}
				~ParseInfo() {}

				map< string, NodePtr > nodeMap; // nodeid --> id-in-graph
				map< string, EdgePtr > edgeMap; // edgeid --> id-in-graph
				map< string, std::tuple<string,string> > edges;
				map< string, std::tuple<string,string,string> > keyMap; // keyid --> (target, type, name)

				vector< std::tuple<string,string> > graphData; // (keyid, value)
				map< string, vector< std::tuple<string,string> > >  nodeDataMap;  // nodeid --> vec((keyid, value))
				map< string, vector< std::tuple<string,string> > >  edgeDataMap;  // edgeid --> vec((keyid, value))

				std::string currentDataKey;
				std::string currentElementKey;
				std::string charData;

				UndirectedGraph* graph;
				ParseEntity currentEntity;
		};
		static void deserializeGraphMLStartTag(void *data, const char* element, const char** attributes);
		static void deserializeGraphMLEndTag(void *data, const char* element);
		static void deserializeGraphMLCharData(void *data, const char* cdata, int len);
		void serializeGraphMLNode(stringstream& out, NodePtr n, map<string, std::tuple<string, string, string> >& attrMap, Indentation indent);
		void serializeGraphMLEdge(stringstream& out, EdgePtr e, map<string, std::tuple<string, string, string> >& attrMap, Indentation indent);

	protected:
		NodeList  m_nodes;
		EdgeList  m_edges;
		unsigned int   m_nextNodeId;
		unsigned int   m_nextEdgeId;
};


class UndirectedGraph::Node : public RandomAttributes {
	friend class UndirectedGraph;

	protected:
		Node(Graph* graph) : m_graph(graph), m_id(0), m_visited(false) {}

	public:
		virtual ~Node();

		// inline void copyAttributes(NodePtr other) {
			// m_attributes = other->m_attributes;
			// m_id = other->m_id;
		// }
		// inline void combineAttributes(NodePtr other) {
			// m_attributes = other->m_attributes;
		// }

		Graph* getGraphPtr() { return m_graph; }

		inline UId getId() const { return m_id; }
		inline void setId(UId id) { m_id = id; }

		EdgePtr linkTo(NodePtr other);
		set<NodePtr> getNeighbors() const;
		set<NodePtr> getNonVisitedNeighbors() const;
		list<EdgePtr>& getEdges();
		const list<EdgePtr>& getEdges() const;
		list<EdgePtr> getNonVisitedEdges() const;
		unsigned int getEdgeCount() const;
		unsigned int getNonVisitedEdgeCount() const;
		void removeEdge(UId);

		inline void remove() { m_graph->removeNode(m_id); }

		inline bool isVisited() const { return m_visited; }
		inline void setVisited(bool v) { m_visited = v; }

		bool operator ==(const Node& other);
		bool operator !=(const Node& other);

	protected:
		Graph*            m_graph;
		UId               m_id;
		list<EdgePtr>     m_edges;
		bool              m_visited;
};

class UndirectedGraph::Edge : public RandomAttributes {
	friend class UndirectedGraph;

	protected:
		Edge(Graph* graph, NodePtr source, NodePtr target) : m_graph(graph), m_id(0),	m_source(source),	m_target(target), m_visited(false) { }
		Edge(Graph* graph, UId id, NodePtr source, NodePtr target) :	m_graph(graph), m_id(id), m_source(source), m_target(target), m_visited(false) {}

	public:
		virtual ~Edge() { /* do not destroy nodes here as graph class destructor handles these */ }

		Graph* getGraphPtr() { return m_graph; }

		inline UId getId() const { return m_id; }
		inline void setId(UId id) { m_id = id; }

		NodePtr getSourceNode();
		NodePtr getTargetNode();

		inline void remove() { m_graph->removeEdge(m_id); }

		inline bool isVisited() const { return m_visited; }
		inline void setVisited(bool state) { m_visited = state; }

		bool operator ==(const Edge& other);
		bool operator !=(const Edge& other);

	protected:
		Graph*      m_graph;
		UId         m_id;
		NodePtr     m_source;
		NodePtr     m_target;
		bool        m_visited;
};

#include "UndirectedGraph.inl"

}


#endif /* UNDIRECTEDGRAPH_H_ */
