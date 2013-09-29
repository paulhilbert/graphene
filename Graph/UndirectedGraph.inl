inline UndirectedGraph::NodePtr UndirectedGraph::addNode() {
	UndirectedGraph::NodePtr node = new Node(this);
	node->m_id = m_nextNodeId++;
	m_nodes.push_back(node);
	return node;
}

inline UndirectedGraph::NodePtr UndirectedGraph::getFirstNode() {
	if (!m_nodes.size()) return NULL;
	return *(m_nodes.begin());
}

inline UndirectedGraph::NodeList& UndirectedGraph::getNodes() {
	return m_nodes;
}

inline UndirectedGraph::EdgeList& UndirectedGraph::getEdges() {
	return m_edges;
}

inline const UndirectedGraph::NodeList& UndirectedGraph::getNodes() const {
	return m_nodes;
}

inline const UndirectedGraph::EdgeList& UndirectedGraph::getEdges() const {
	return m_edges;
}

inline UndirectedGraph::NodeList UndirectedGraph::getNonVisitedNodes() const {
	UndirectedGraph::NodeList nodeList;
	for (UndirectedGraph::NodeList::const_iterator it = m_nodes.begin(); it != m_nodes.end(); ++it) {
		if ((*it)->isVisited()) continue;
		nodeList.push_back(*it);
	}
	return nodeList;
}

inline UndirectedGraph::EdgeList UndirectedGraph::getNonVisitedEdges() const {
	UndirectedGraph::EdgeList edgeList;
	for (UndirectedGraph::EdgeList::const_iterator it = m_edges.begin(); it != m_edges.end(); ++it) {
		if ((*it)->isVisited()) continue;
		edgeList.push_back(*it);
	}
	return edgeList;
}

inline UndirectedGraph::NodePtr UndirectedGraph::getNodeForId(UndirectedGraph::UId id) {
	NodePtr result = NULL;
	for (list<NodePtr>::iterator it = m_nodes.begin(); it != m_nodes.end(); ++it) {
		if ((*it)->getId() == id) {
			result = *it;
			break;
		}
	}

	return result;
}

inline UndirectedGraph::EdgePtr UndirectedGraph::getEdgeForId(UndirectedGraph::UId id) {
	EdgePtr result = NULL;
	for (list<EdgePtr>::iterator it = m_edges.begin(); it != m_edges.end(); ++it) {
		if ((*it)->getId() == id) {
			result = *it;
			break;
		}
	}

	return result;
}

inline unsigned int UndirectedGraph::getNodeCount() const {
	return m_nodes.size();
}

inline unsigned int UndirectedGraph::getEdgeCount() const {
	return m_edges.size();
}

inline unsigned int UndirectedGraph::getMaxOutdeg() const {
	unsigned int result = 0;
	for (NodeList::const_iterator it = m_nodes.begin(); it != m_nodes.end(); ++it) {
		unsigned int degree = (*it)->getEdgeCount();
		if (degree > result) result = degree;
	}
	return result;
}

inline unsigned int UndirectedGraph::getNonVisitedNodeCount() const {
	unsigned int nodeCount = 0;
	for (UndirectedGraph::NodeList::const_iterator it = m_nodes.begin(); it != m_nodes.end(); ++it) {
		if (!(*it)->isVisited()) ++nodeCount;
	}
	return nodeCount;
}

inline unsigned int UndirectedGraph::getNonVisitedEdgeCount() const {
	unsigned int edgeCount = 0;
	for (UndirectedGraph::EdgeList::const_iterator it = m_edges.begin(); it != m_edges.end(); ++it) {
		if (!(*it)->isVisited()) ++edgeCount;
	}
	return edgeCount;
}

inline void UndirectedGraph::copyNodeAttributes(NodePtr from, NodePtr to) {
	combineNodeAttributes(from, to);
	to->setId(from->getId());
}

inline void UndirectedGraph::combineNodeAttributes(NodePtr from, NodePtr to) {
	to->getAttributes() = from->getAttributes();
}

inline void UndirectedGraph::copyEdgeAttributes(EdgePtr from, EdgePtr to) {
	combineEdgeAttributes(from, to);
	to->setId(from->getId());
}

inline void UndirectedGraph::combineEdgeAttributes(EdgePtr from, EdgePtr to) {
	to->getAttributes() = from->getAttributes();
}

inline void UndirectedGraph::insertEdge(UndirectedGraph::EdgePtr edge) {
	edge->m_id = m_nextEdgeId++;
	m_edges.push_back(edge);
}

inline void UndirectedGraph::insertNode(NodePtr node) {
	m_nodes.push_back(node);
}

inline void UndirectedGraph::resetNodeIds() {
	unsigned int id = 0;
	for (NodeList::iterator it = m_nodes.begin(); it != m_nodes.end(); ++it) {
		(*it)->m_id = id;
		++id;
	}
	m_nextNodeId = m_nodes.size();
}


// NODE //

inline UndirectedGraph::EdgePtr UndirectedGraph::Node::linkTo(UndirectedGraph::NodePtr other) {
	if (*this == *other)	return NULL;

	EdgePtr edge = new Edge(m_graph, this, other);
	m_edges.push_back(edge);
	other->m_edges.push_back(edge);
	m_graph->insertEdge(edge);

	return edge;
}

inline set<UndirectedGraph::NodePtr> UndirectedGraph::Node::getNeighbors() const {
	set<NodePtr> result;
	for(list<EdgePtr>::const_iterator it = m_edges.begin(); it != m_edges.end(); ++it) {
		if ((*it)->getSourceNode()->getId() != this->getId()) result.insert((*it)->getSourceNode());
		else if ((*it)->getTargetNode()->getId() != this->getId()) result.insert((*it)->getTargetNode());
	}
	return result;
}

inline set<UndirectedGraph::NodePtr> UndirectedGraph::Node::getNonVisitedNeighbors() const {
	set<NodePtr> result;
	for(list<EdgePtr>::const_iterator it = m_edges.begin(); it != m_edges.end(); ++it) {
		NodePtr neighbor = NULL;
		if ((*it)->getSourceNode()->getId() != this->getId()) neighbor = (*it)->getSourceNode();
		else if ((*it)->getTargetNode()->getId() != this->getId()) neighbor = (*it)->getTargetNode();
		if (!neighbor->isVisited()) result.insert(neighbor);
	}
	return result;
}

inline list<UndirectedGraph::EdgePtr>& UndirectedGraph::Node::getEdges() {
	return m_edges;
}

inline const list<UndirectedGraph::EdgePtr>& UndirectedGraph::Node::getEdges() const {
	return m_edges;
}

inline list<UndirectedGraph::EdgePtr> UndirectedGraph::Node::getNonVisitedEdges() const {
	list<UndirectedGraph::EdgePtr> edgeList;
	for (list<UndirectedGraph::EdgePtr>::const_iterator it = m_edges.begin(); it != m_edges.end(); ++it) {
		if ((*it)->isVisited()) continue;
		edgeList.push_back(*it);
	}
	return edgeList;
}

inline unsigned int UndirectedGraph::Node::getEdgeCount() const {
	return m_edges.size();
}

inline unsigned int UndirectedGraph::Node::getNonVisitedEdgeCount() const {
	unsigned int edgeCount = 0;
	for (list<UndirectedGraph::EdgePtr>::const_iterator it = m_edges.begin(); it != m_edges.end(); ++it) {
		if (!(*it)->isVisited()) ++edgeCount;
	}
	return edgeCount;
}

inline void UndirectedGraph::Node::removeEdge(UId id) {
	list<EdgePtr>::iterator eIt = m_edges.begin();
	while (eIt != m_edges.end()) {
		if ((*eIt)->getId() == id) {
			eIt = m_edges.erase(eIt);
			continue;
		}
		++eIt;
	}
}

inline bool UndirectedGraph::Node::operator ==(const UndirectedGraph::Node& other) {
	return (m_id == other.m_id);
}

inline bool UndirectedGraph::Node::operator !=(const UndirectedGraph::Node& other) {
	return !operator==(other);
}


// EDGE //

inline UndirectedGraph::NodePtr UndirectedGraph::Edge::getSourceNode() {
	return m_source;
}

inline UndirectedGraph::NodePtr UndirectedGraph::Edge::getTargetNode() {
	return m_target;
}

inline bool UndirectedGraph::Edge::operator ==(const UndirectedGraph::Edge& other) {
	return (m_id == other.m_id);
}

inline bool UndirectedGraph::Edge::operator !=(const UndirectedGraph::Edge& other) {
	return !operator==(other);
}
