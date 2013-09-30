namespace Graph {

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
inline Boost<Type, NodeProp, EdgeProp, GraphProp>::Boost() : Base() {
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
inline Boost<Type, NodeProp, EdgeProp, GraphProp>::~Boost() {
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
inline typename Boost<Type, NodeProp, EdgeProp, GraphProp>::NodeHandle Boost<Type, NodeProp, EdgeProp, GraphProp>::addNode() {
	return ::boost::add_vertex(*this);
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
inline typename Boost<Type, NodeProp, EdgeProp, GraphProp>::EdgeAddResult  Boost<Type, NodeProp, EdgeProp, GraphProp>::addEdge(NodeHandle src, NodeHandle tgt) {
	return ::boost::add_edge(src, tgt, *this);
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
inline typename Boost<Type, NodeProp, EdgeProp, GraphProp>::NodeRange  Boost<Type, NodeProp, EdgeProp, GraphProp>::nodes() {
	return NodeRange(::boost::vertices(*this));
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
inline typename Boost<Type, NodeProp, EdgeProp, GraphProp>::EdgeRange  Boost<Type, NodeProp, EdgeProp, GraphProp>::edges() {
	return EdgeRange(::boost::edges(*this));
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
inline typename Boost<Type, NodeProp, EdgeProp, GraphProp>::OutEdgeRange  Boost<Type, NodeProp, EdgeProp, GraphProp>::outEdges(NodeHandle node) {
	return OutEdgeRange(::boost::out_edges(node, *this));
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
inline typename Boost<Type, NodeProp, EdgeProp, GraphProp>::NodeHandle Boost<Type, NodeProp, EdgeProp, GraphProp>::source(EdgeHandle edge) {
	return ::boost::source(edge, *this);
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
inline typename Boost<Type, NodeProp, EdgeProp, GraphProp>::NodeHandle Boost<Type, NodeProp, EdgeProp, GraphProp>::target(EdgeHandle edge) {
	return ::boost::target(edge, *this);
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
inline typename Boost<Type, NodeProp, EdgeProp, GraphProp>::EdgeNodes  Boost<Type, NodeProp, EdgeProp, GraphProp>::nodes(EdgeHandle edge) {
	return {source(edge), target(edge)};
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
inline unsigned int Boost<Type, NodeProp, EdgeProp, GraphProp>::numNodes() const {
	return ::boost::num_vertices(*this);
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
inline unsigned int Boost<Type, NodeProp, EdgeProp, GraphProp>::numEdges() const {
	return ::boost::num_edges(*this);
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
inline void Boost<Type, NodeProp, EdgeProp, GraphProp>::forallNodes(NodeVisitor visitor) {
	for (auto n : nodes()) visitor(*this, n);
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
inline void Boost<Type, NodeProp, EdgeProp, GraphProp>::forallEdges(EdgeVisitor visitor) {
	for (auto e : edges()) visitor(*this, e);
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
inline void Boost<Type, NodeProp, EdgeProp, GraphProp>::forallOutEdges(NodeHandle node, EdgeVisitor visitor) {
	for (auto e : outEdges(node)) visitor(*this, e);
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
inline void Boost<Type, NodeProp, EdgeProp, GraphProp>::pruneEdges(EdgePredicate pred) {
	::boost::remove_edge_if(std::bind(pred, std::ref(*this), std::placeholders::_1), *this);
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
unsigned int Boost<Type, NodeProp, EdgeProp, GraphProp>::connectedComponents(Coloring& components) const {
	components.resize(numNodes());
	return ::boost::connected_components(*this, &components[0]);
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
typename Boost<Type,NodeProp,EdgeProp,GraphProp>::Components Boost<Type,NodeProp,EdgeProp,GraphProp>::connectedComponents() {
	Coloring coloring;
	auto count = connectedComponents(coloring);
	Components components(count);
	unsigned int v = 0;
	for (auto vertex : nodes()) {
		components[coloring[v++]].push_back(vertex);
	}
	return components;
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
typename Boost<Type,NodeProp,EdgeProp,GraphProp>::EdgeHandles Boost<Type,NodeProp,EdgeProp,GraphProp>::edges(Component& component) {
	EdgeHandles result;
	forallEdges([&] (Boost& g, EdgeHandle e) {
		NodeHandle src, tgt;
		std::tie(src, tgt) = nodes(e);
		if (std::find(component.begin(), component.end(), src) != component.end()
		&&  std::find(component.begin(), component.end(), tgt) != component.end()) result.push_back(e);
	});
	return result;
}

template <class Type, class NodeProp, class EdgeProp, class GraphProp>
template <class Archive>
inline void Boost<Type, NodeProp, EdgeProp, GraphProp>::serialize(Archive& ar, const unsigned int version) {
	Base* base = dynamic_cast<Base*>(this);
	ar & (*base);
}

} // Graph
