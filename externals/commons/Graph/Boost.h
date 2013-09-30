#ifndef GRAPHBOOST_H_
#define GRAPHBOOST_H_

#include <memory>
#include <tuple>
#include <functional>

#include <Generic/Range.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/graph/connected_components.hpp>

#include <boost/serialization/access.hpp>

namespace Graph {

// use these as Type param below
struct DirTag   { typedef ::boost::directedS      type; };
struct UnDirTag { typedef ::boost::undirectedS    type; };
struct BiDirTag { typedef ::boost::bidirectionalS type; };

typedef ::boost::no_property NullProp;

// shortener t'defs
template <class Type, class NodeProp, class EdgeProp, class GraphProp>
using GraphBase = ::boost::adjacency_list<boost::vecS, boost::vecS, typename Type::type, NodeProp, EdgeProp, GraphProp>;
template <class Base>
using GraphTraits = ::boost::graph_traits<Base>;



template <class Type, class NodeProp = NullProp, class EdgeProp = NullProp, class GraphProp = NullProp>
class Boost : public GraphBase<Type, NodeProp, EdgeProp, GraphProp> {
	public:
		typedef std::shared_ptr<Boost> Ptr;
		typedef std::weak_ptr<Boost>   WPtr;

		typedef GraphBase<Type, NodeProp, EdgeProp, GraphProp>   Base;
		typedef GraphTraits<Base>                                Traits;

		typedef typename Traits::vertex_descriptor               NodeHandle;
		typedef typename Traits::edge_descriptor                 EdgeHandle;

		typedef std::vector<NodeHandle>                          NodeHandles;
		typedef std::vector<EdgeHandle>                          EdgeHandles;

		typedef typename Traits::vertex_iterator                 NodeIter;
		typedef typename Traits::edge_iterator                   EdgeIter;
		typedef typename Traits::out_edge_iterator               OutEdgeIter;
		typedef Generic::Range<NodeIter>                         NodeRange;
		typedef Generic::Range<EdgeIter>                         EdgeRange;
		typedef Generic::Range<OutEdgeIter>                      OutEdgeRange;

		typedef std::pair<EdgeHandle, bool>                      EdgeAddResult;
		typedef std::pair<NodeHandle, NodeHandle>                EdgeNodes;

		typedef std::function<void (Boost&, NodeHandle)>         NodeVisitor;
		typedef std::function<void (Boost&, EdgeHandle)>         EdgeVisitor;
		typedef std::function<bool (Boost&, EdgeHandle)>         EdgePredicate;

		typedef std::vector<unsigned int>                        Coloring;
		typedef std::vector<NodeHandle>                          Component;
		typedef std::vector<Component>                           Components;

	public:
		Boost();
		virtual ~Boost();

		NodeHandle     addNode();
		EdgeAddResult  addEdge(NodeHandle src, NodeHandle tgt);

		NodeRange      nodes();
		EdgeRange      edges();

		OutEdgeRange   outEdges(NodeHandle node);
		NodeHandle     source(EdgeHandle edge);
		NodeHandle     target(EdgeHandle edge);
		EdgeNodes      nodes(EdgeHandle edge);

		unsigned int   numNodes() const;
		unsigned int   numEdges() const;

		void           forallNodes(NodeVisitor visitor);
		void           forallEdges(EdgeVisitor visitor);
		void           forallOutEdges(NodeHandle node, EdgeVisitor visitor);

		void           pruneEdges(EdgePredicate pred);

		// connected components
		unsigned int   connectedComponents(Coloring& components) const;
		Components     connectedComponents();
		EdgeHandles    edges(Component& component);

	protected:
		// serialization
		friend class boost::serialization::access;
		template <class Archive>
		void serialize(Archive& ar, const unsigned int version);
};

} // Graph

#include "Boost.inl"


#endif /* GRAPHBOOST_H_ */
