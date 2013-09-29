template <class Graph>
typename Sample<Graph>::NodePtr Sample<Graph>::randomNode(Graph& graph) {
	NodeList nodeList = graph.getNodes();
	if (!nodeList.size()) return NULL;

	typename NodeList::iterator it = nodeList.begin();
	RNG* rng = Random::RNG::instance();
	advance(it, rng->uniformAB<unsigned int>(0, nodeList.size()));
	return *it;
}

template <class Graph>
typename Sample<Graph>::GraphPtr Sample<Graph>::mergeMultiEdges(Graph& graph) {
	// we want to return a completely isolated new graph
	GraphPtr result(new Graph());
	// copy nodes first
	Iterate<Graph>::iterateNodes(graph, [&result] (NodePtr n) {
		NodePtr x = result->addNode();
		Graph::copyNodeAttributes(n, x);
	});

	// we need a fast way to determine a similiar edge
	// we found earlier, so we use a map
	// (n0.id,n1.id) -> EdgePtr ...
	typedef pair<typename Graph::UId, typename Graph::UId> IdPair;
	typedef map<IdPair, EdgePtr> EdgeMap;
	EdgeMap edgeMap;
	// ... with n0.id < n1.id
	// and fill it by walking through all edges
	EdgeList edgeList = graph.getEdges();
	for (typename EdgeList::iterator it = edgeList.begin(); it != edgeList.end(); ++it) {
		NodePtr n0 = (*it)->getSourceNode();
		NodePtr temp = (*it)->getTargetNode();
		NodePtr n1;
		if (temp->getId() < n0->getId()) {
			n1 = n0;
			n0 = temp;
		} else {
			n1 = temp;
		}
		// we now established the invariant n0.id < n1.id
		// do a lookup in our map
		IdPair idPair = IdPair(n0->getId(), n1->getId());
		typename EdgeMap::iterator f = edgeMap.find(idPair);
		if (f == edgeMap.end()) {
			// n0 and n1 are not connected yet
			NodePtr s = result->getNodeForId(n0->getId());
			NodePtr t = result->getNodeForId(n1->getId());
			// create and remember edge
			EdgePtr e = s->linkTo(t);
			Graph::copyEdgeAttributes(*it, e);
			edgeMap[idPair] = e;
		} else {
			Graph::combineEdgeAttributes(*it, f->second);
		}
	}

	return result;
}

template <class Graph>
vector<typename Sample<Graph>::GraphPtr> Sample<Graph>::sampleSubgraphs(Graph& graph,	unsigned int count, unsigned int size, SampleMethod method, SampleSizePolicy sizePolicy, EdgeAcceptor edgeAcceptor) {
	// we will work on components
	vector<GraphPtr> components = Decompose<Graph>::connectedComponents(graph);
	// copy all components to considered, that have a nodecount >= size
	vector<GraphPtr> considered;
	std::for_each(components.begin(), components.end(), [&](GraphPtr g) {
		if (g->getNodes().size() >= size) considered.push_back(g);
	});

	vector<GraphPtr> result;
	// we won't need to start at all if there are no components left
	if (!considered.size()) return result;

	// prepare continue predicate
	std::function<bool (unsigned int nVisited, unsigned int depth, unsigned int maxDepth)> contPredicate;
	switch (sizePolicy) {
		case EQUAL: contPredicate = [size](unsigned int n, unsigned int d, unsigned int md)->bool { return n < size; }; break;
		default:   	contPredicate = [size](unsigned int n, unsigned int d, unsigned int md)->bool {
							RNG* rng = RNG::instance();
							return n < size || rng->uniform01<float>() < 0.4f;
						};
	}

	// use random index into components to sample count subgraphs
	RNG* rng = RNG::instance();
	RNG::Traits<unsigned int>::GenAB pickComponent = rng->uniformABGen<unsigned int>(0, considered.size());
	unsigned int triedWithoutSuccess = 0; // avoid hell

	while (result.size() < count) {
		int randC = pickComponent();
		GraphPtr sample(new Graph());
		bool success = false;
		switch (method) {
			case FOREST_FIRE: success = Traverse<Graph>::forestFire(
				*(considered[randC]),
				// node visitor
				[&sample] (NodePtr n) {
					NodePtr x = sample->getNodeForId(n->getId());
					if (!x) {
						x = sample->addNode();
						Graph::copyNodeAttributes(n, x);
					}
				},
				// edge visitor
				[&sample] (EdgePtr edge) {
					NodePtr source = edge->getSourceNode();
					NodePtr target = edge->getTargetNode();
					NodePtr x = sample->getNodeForId(source->getId());
					NodePtr y = sample->getNodeForId(target->getId());
					if (!x) { x = sample->addNode(); Graph::copyNodeAttributes(source, x); }
					if (!y) { y = sample->addNode(); Graph::copyNodeAttributes(target, y); }
					EdgePtr e = x->linkTo(y);
					if (e) Graph::copyEdgeAttributes(edge,e);
				},
				randomNode(*(considered[randC])), 0.5f, contPredicate, edgeAcceptor);
				break;
			default:           success = Traverse<Graph>::randomWalk(
				*(considered[randC]),
				// node visitor
				[&sample] (NodePtr n) {
					NodePtr x = sample->getNodeForId(n->getId());
					if (!x) {
						x = sample->addNode();
						Graph::copyNodeAttributes(n, x);
					}
				},
				// edge visitor
				[&sample] (EdgePtr edge) {
					NodePtr source = edge->getSourceNode();
					NodePtr target = edge->getTargetNode();
					NodePtr x = sample->getNodeForId(source->getId());
					NodePtr y = sample->getNodeForId(target->getId());
					if (!x) { x = sample->addNode(); Graph::copyNodeAttributes(source, x); }
					if (!y) { y = sample->addNode(); Graph::copyNodeAttributes(target, y); }
					EdgePtr e = x->linkTo(y);
					if (e) Graph::copyEdgeAttributes(edge,e);
				},
				randomNode(*(considered[randC])), contPredicate, edgeAcceptor);
		}
		Iterate<Graph>::iterateNodes( *(considered[randC]), bind(&Node::setVisited, std::placeholders::_1, false) );
		if ((triedWithoutSuccess > 50) || success) {
			result.push_back(sample);
			triedWithoutSuccess = 0;
		}
		else ++triedWithoutSuccess;
	}

	return result;
}

template <class Graph>
typename Sample<Graph>::GraphPtr Sample<Graph>::kRingNeighborhood(Graph& graph, NodePtr node, unsigned int k) {
	GraphPtr neighborhood(new Graph());

	// node visitor that clones node to new graph if necessary
	// (actually this is only necessary for the case where node is isolated since the edge visitor will
	// clone all nodes)
	auto cloneNode = [&neighborhood] (NodePtr n) {
		NodePtr newNode = neighborhood->getNodeForId(n->getId());
		if (!newNode) {
			newNode = neighborhood->addNode();
			Graph::copyNodeAttributes(n, newNode);
		}
	};

	// edge visitor that clones edge to new graph
	auto cloneEdge = [&neighborhood, &cloneNode] (EdgePtr e) {
		NodePtr source = e->getSourceNode();
		NodePtr target = e->getTargetNode();
		cloneNode(source);
		cloneNode(target); // most probably at least the target node does not exist yet
		NodePtr newSource = neighborhood->getNodeForId(source->getId());
		NodePtr newTarget = neighborhood->getNodeForId(target->getId());
		EdgePtr newEdge = newSource->linkTo(newTarget);
		if (newEdge) Graph::copyEdgeAttributes(e, newEdge);
	};

	// depth-limited breadth-first-search with clone visitors will yield our neighorhood graph
	Traverse<Graph>::breadthFirstTraverse(cloneNode, cloneEdge, node, k);

	return neighborhood;
}
