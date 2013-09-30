template <class Graph>
inline void Traverse<Graph>::traverse(NodeVisitor nodeVisitor, EdgeVisitor edgeVisitor, NodePtr start, QueueGet getNext, QueueInsert putNext, unsigned int maxDepth) {
	traverse(nodeVisitor, edgeVisitor, start, getNext, putNext, 0, maxDepth);
}

template <class Graph>
inline void Traverse<Graph>::depthFirstTraverse(NodeVisitor nodeVisitor, EdgeVisitor edgeVisitor, NodePtr start, unsigned int maxDepth) {
	TraverseQueue<Graph> q;
	QueueGet    getNext = std::bind(&TraverseQueue<Graph>::getNext, &q, TraverseQueue<Graph>::LIFO);
	QueueInsert putNext = std::bind(&TraverseQueue<Graph>::putNext, &q, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, TraverseQueue<Graph>::LIFO);
	traverse(nodeVisitor, edgeVisitor, start, getNext, putNext, maxDepth);
}

template <class Graph>
inline void Traverse<Graph>::breadthFirstTraverse(NodeVisitor nodeVisitor, EdgeVisitor edgeVisitor, NodePtr start, unsigned int maxDepth) {
	TraverseQueue<Graph> q;
	QueueGet    getNext = std::bind(&TraverseQueue<Graph>::getNext, &q, TraverseQueue<Graph>::FIFO);
	QueueInsert putNext = std::bind(&TraverseQueue<Graph>::putNext, &q, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, TraverseQueue<Graph>::FIFO);
	traverse(nodeVisitor, edgeVisitor, start, getNext, putNext, maxDepth);
}

template <class Graph>
inline bool Traverse<Graph>::randomWalk(Graph& graph, NodeVisitor nodeVisitor, EdgeVisitor edgeVisitor, NodePtr start, ContinuePredicate contPredicate, EdgeAcceptor edgeAcceptor) {
	unsigned int nodesVisited = 0;
	unsigned int maxDepth = 0;
	return randomWalk(graph, nodeVisitor, edgeVisitor, start, start, contPredicate, edgeAcceptor, nodesVisited, 0, maxDepth);
}

template <class Graph>
inline bool Traverse<Graph>::forestFire(Graph& graph, NodeVisitor nodeVisitor, EdgeVisitor edgeVisitor, NodePtr start, float fwBurnProb, ContinuePredicate contPredicate, EdgeAcceptor edgeAcceptor) {
	unsigned int maxDepth = 0;
	unsigned int nodesVisited = 0;
	
	RNG* rng = RNG::instance();
	RNG::Traits<unsigned int>::GenGeom pRand = rng->geometricGen<unsigned int>(1.f - fwBurnProb);

	TraverseQueue<Graph> queue;

	return forestFire(graph, nodeVisitor, edgeVisitor, start, start, queue, pRand, contPredicate, edgeAcceptor, nodesVisited, 0, maxDepth);
}

template <class Graph>
inline void Traverse<Graph>::traverse(NodeVisitor nodeVisitor, EdgeVisitor edgeVisitor, NodePtr start, QueueGet getNext, QueueInsert putNext, unsigned int depth, unsigned int maxDepth) {
	if (depth > maxDepth) return;
	start->setVisited(true);
	if (nodeVisitor) nodeVisitor(start);

	EdgeList neighborEdges = start->getEdges();
	for (typename EdgeList::iterator it = neighborEdges.begin(); it != neighborEdges.end(); ++it) {
		if ((*it)->getSourceNode()->isVisited() && (*it)->getTargetNode()->isVisited()) continue;
		putNext(start, *it, depth+1);
	}
	NodePtr target; EdgeList connections; unsigned int nextDepth;
	tie(ignore, target, connections, nextDepth) = getNext();
	while (target && target->isVisited()) {
		tie(ignore, target, connections, nextDepth) = getNext();
	}
	if (!target) return;
	// else
	if (edgeVisitor) {
		for (typename EdgeList::iterator edgeIt = connections.begin(); edgeIt != connections.end(); ++edgeIt) {
			edgeVisitor(*edgeIt);
		}
	}
	traverse(nodeVisitor, edgeVisitor, target, getNext, putNext, nextDepth, maxDepth);
}

template <class Graph>
bool Traverse<Graph>::randomWalk(Graph& graph, NodeVisitor nodeVisitor, EdgeVisitor edgeVisitor, NodePtr start, NodePtr current, ContinuePredicate contPredicate, EdgeAcceptor edgeAcceptor, unsigned int& nodesVisited, unsigned int depth, unsigned int& maxDepth) {
	if (depth > maxDepth) maxDepth = depth;
	if (!current->isVisited()) {
		if (nodeVisitor) nodeVisitor(current);
		current->setVisited(true);
		++nodesVisited;
	}
	if (!contPredicate(nodesVisited, depth, maxDepth)) {
		return true;
	}

	RNG* rng = RNG::instance();
	if (start != current && rng->uniform01<float>() < 0.15f ) {
		return randomWalk(graph, nodeVisitor, edgeVisitor, start, start, contPredicate, edgeAcceptor, nodesVisited, 0, maxDepth);
	}

	EdgeList neighborEdges = current->getEdges();
	if (!neighborEdges.size()) return false;
	typename EdgeList::iterator it = neighborEdges.begin();

	advance(it, rng->uniformAB<unsigned int>(0, neighborEdges.size()));
	// advance further until a non-visited node is found
	// if we tried all without success jump back to start
	// or fail if we already are there
	unsigned int considered = 1;
	NodePtr neighbor = (*it)->getSourceNode() != current ? (*it)->getSourceNode() : (*it)->getTargetNode();
	while (((edgeAcceptor ? (!edgeAcceptor(*it)) : false) || neighbor->isVisited()) && considered < neighborEdges.size()) {
		if (++it == neighborEdges.end()) it = neighborEdges.begin();
		++considered;
		neighbor = (*it)->getSourceNode() != current ? (*it)->getSourceNode() : (*it)->getTargetNode();
	}
	if (considered >= neighborEdges.size() && (neighbor->isVisited() || (edgeAcceptor ? (!edgeAcceptor(*it)) : false))) {
		if (current == start) return false;
		return randomWalk(graph, nodeVisitor, edgeVisitor, start, start, contPredicate, edgeAcceptor, nodesVisited, 0, maxDepth);
	}
	// at this point exists an unvisited random neighbor node pointed at by it
	// recurse to this node
	if (edgeVisitor) edgeVisitor(*it);
	return randomWalk(graph, nodeVisitor, edgeVisitor, start, neighbor, contPredicate, edgeAcceptor, nodesVisited, depth+1, maxDepth);
}

template <class Graph>
bool Traverse<Graph>::forestFire(Graph& graph, NodeVisitor nodeVisitor, EdgeVisitor edgeVisitor, NodePtr start, NodePtr current, TraverseQueue<Graph>& queue, RNGGeom pRand, ContinuePredicate contPredicate, EdgeAcceptor edgeAcceptor, unsigned int& nodesVisited, unsigned int depth, unsigned int& maxDepth) {
	if (depth > maxDepth) maxDepth = depth;
	if (!current->isVisited()) {
		if (nodeVisitor) nodeVisitor(current);
		current->setVisited(true);
		++nodesVisited;
	}
	if (!contPredicate(nodesVisited, depth, maxDepth)) {
		return true;
	}

	EdgeList neighborEdges = current->getEdges();
	typename EdgeList::iterator it = neighborEdges.begin();

	RNG* rng = RNG::instance();
	advance(it, rng->uniformAB<unsigned int>(0, neighborEdges.size()));
	// gather x unvisited out-edges to different nodes
	unsigned int x = pRand();
	if (x > neighborEdges.size()) x = neighborEdges.size();

	unsigned int considered = 0;
	unsigned int lastSize = queue.getQueueSize();
	while (considered < neighborEdges.size() && (queue.getQueueSize() - lastSize) < x) {
		if (edgeAcceptor ? edgeAcceptor(*it) : true)
			queue.putNext(current, *it, depth+1, Algorithm::TraverseQueue<Graph>::FIFO);
		++considered;
		if (++it == neighborEdges.end()) it = neighborEdges.begin();
	}

	// at this point we have min(#unvisited_neighbors,x) targets in our queue
	// if we have at least one node in there visit all edges and recurse
	// else try again from the beginning
	if (!queue.getQueueSize() && contPredicate(nodesVisited, depth, maxDepth)) {
		if (current == start) return false;
		forestFire(graph, nodeVisitor, edgeVisitor, start, start, queue, pRand, contPredicate, edgeAcceptor, nodesVisited, 0, maxDepth);
	}
	NodePtr target; list<EdgePtr> connections;
	tie(ignore, target, connections, ignore) = queue.getNext(Algorithm::TraverseQueue<Graph>::FIFO);
	while (target && target->isVisited()) {
		tie(ignore, target, connections, ignore) = queue.getNext(Algorithm::TraverseQueue<Graph>::FIFO);
	}
	if (!target) return false;
	// else
	if (edgeVisitor) {
		for (typename EdgeList::iterator edgeIt = connections.begin(); edgeIt != connections.end(); ++edgeIt) {
			edgeVisitor(*edgeIt);
		}
	}
	
	return forestFire(graph, nodeVisitor, edgeVisitor, start, target, queue, pRand, contPredicate, edgeAcceptor, nodesVisited, depth+1, maxDepth);
}

template <class Graph>
inline void TraverseQueue<Graph>::putNext(NodePtr current, EdgePtr e, unsigned int depth, DequeVariant variant) {
	NodePtr s = e->getSourceNode();
	NodePtr t = e->getTargetNode();
	NodePtr other;
	if (s != current) {
		if (t != current) return; // we failed
		other = s;
	} else if (t != current) {
		other = t;
	} else return; // we failed

	if (other->isVisited()) return;

	// check if front/back already are a connection like this
	if (m_queue.size()) {
		typename deque<QueueElement>::iterator possDupe = (variant == LIFO) ? m_queue.begin() : (m_queue.end()-1);
		if (std::get<0>(*possDupe) == current && std::get<1>(*possDupe) == other) {
			// simply push the edge
			std::get<2>(*possDupe).push_back(e);
			return;
		}
	}

	// else create new element
	QueueElement newElement(current, other, MultiEdge(), depth);
	std::get<2>(newElement).push_back(e);
	switch (variant) {
		case LIFO: m_queue.push_front(newElement); break;
		case FIFO: m_queue.push_back(newElement);  break;
		default: return;
	}
}

template <class Graph>
inline typename TraverseQueue<Graph>::QueueElement TraverseQueue<Graph>::getNext(DequeVariant variant) {
	if (!m_queue.size() || !m_queue.size()) return QueueElement((NodePtr)NULL, (NodePtr)NULL, MultiEdge(), 0);
	QueueElement result;
	switch (variant) {
		case LIFO:
		case FIFO: result = m_queue.front(); m_queue.pop_front(); break;
		default: result = QueueElement((NodePtr)NULL, (NodePtr)NULL, MultiEdge(), 0);
	}
	return result;
}

template <class Graph>
inline unsigned int TraverseQueue<Graph>::getQueueSize() const {
	return m_queue.size();
}
