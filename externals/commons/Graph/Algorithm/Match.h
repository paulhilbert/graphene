#ifndef MATCH_H_
#define MATCH_H_

#include <fstream>
using std::ofstream;

#include <boost/optional.hpp>
#include <boost/none.hpp>
using boost::optional;
using boost::none;

#include <tuple>

#include <boost/date_time.hpp>
using namespace boost::posix_time;

#include "Iterate.h"
#include "Sample.h"
#include "Decompose.h"

// libvf matching
#ifdef _WIN32
#include <match.h>
#include <argraph.h>
#include <argedit.h>
#include <vf2_state.h>
#include <vf2_mono_state.h>
#include <vf2_sub_state.h>
#else
#include <libvf/match.h>
#include <libvf/argraph.h>
#include <libvf/argedit.h>
#include <libvf/vf2_state.h>
#include <libvf/vf2_mono_state.h>
#include <libvf/vf2_sub_state.h>
#endif

namespace Graph {
namespace Algorithm {

// VFlib didn't catch that functor idea, so here is what it deserves
template <class Entity>
class Comparator : public VF::AttrComparator {
	public:
		Comparator(std::function<bool (Entity, Entity)> compare, time_duration maxDuration) : m_compare(compare), m_maxDuration(maxDuration) {
			m_startTime = microsec_clock::local_time();
		}

		virtual bool compatible(void* query_pointer, void* target_pointer) {
			// Make RoomNodes out of the pointers we received.
			Entity q = static_cast<Entity>(query_pointer);
			Entity t = static_cast<Entity>(target_pointer);
			ptime now(microsec_clock::local_time());
			if ((now - m_startTime) > m_maxDuration)
				return false;

			return m_compare ? m_compare(q, t) : true;
		}

		inline void resetStartTime() {
			m_startTime = microsec_clock::local_time();
		}

	protected:
		std::function<bool (Entity, Entity)> m_compare;
		ptime m_startTime;
		time_duration m_maxDuration;
};


template <class Graph>
struct Match {
	typedef typename Graph::GraphPtr  GraphPtr;
	typedef typename Graph::Node      Node;
	typedef typename Graph::NodePtr   NodePtr;
	typedef typename Graph::NodeList  NodeList;
	typedef typename Graph::Edge      Edge;
	typedef typename Graph::EdgePtr   EdgePtr;
	typedef typename Graph::EdgeList  EdgeList;
	typedef typename Graph::UId       UId;
	typedef map<UId, UId>             IdMap;
	typedef vector<IdMap>             IsomorphyMap;
	typedef vector<UId>               IdVec;
	typedef std::tuple<IsomorphyMap&, IdVec&, IdVec&, unsigned int&, unsigned int> VisitorData;


	static IsomorphyMap isomorphism(Graph& query, Graph& target, unsigned int maxNumMatches, const time_duration maxDuration = pos_infin, std::function<bool (NodePtr, NodePtr)> nodeCompare = NULL, std::function<bool (EdgePtr, EdgePtr)> edgeCompare = NULL, bool doMerge = true) {
		return mainIsomorphism<VF::VF2State>(query, target, maxNumMatches, maxDuration, nodeCompare, edgeCompare, doMerge);
	}
	static IsomorphyMap monomorphism(Graph& query, Graph& target, unsigned int maxNumMatches, const time_duration maxDuration = pos_infin, std::function<bool (NodePtr, NodePtr)> nodeCompare = NULL, std::function<bool (EdgePtr, EdgePtr)> edgeCompare = NULL, bool doMerge = true) {
		return mainIsomorphism<VF::VF2MonoState>(query, target, maxNumMatches, maxDuration, nodeCompare, edgeCompare, doMerge);
	}
	static IsomorphyMap subgraphIsomorphism(Graph& query, Graph& target, unsigned int maxNumMatches, const time_duration maxDuration = pos_infin, std::function<bool (NodePtr, NodePtr)> nodeCompare = NULL, std::function<bool (EdgePtr, EdgePtr)> edgeCompare = NULL, bool doMerge = true) {
		return mainIsomorphism<VF::VF2SubState>(query, target, maxNumMatches, maxDuration, nodeCompare, edgeCompare, doMerge);
	}

protected:

	static bool matchVisitor(int numNodes, node_id qIds[], node_id tIds[], void* data) {
		VisitorData* info = static_cast<VisitorData*>(data);

		unsigned int& numMatches = std::get<3>(*info);
		unsigned int  maxNumMatches = std::get<4>(*info);
		++numMatches;

		map<UId, UId> nodeMap;

		for (int i=0; i < numNodes; ++i) {
			UId qId = std::get<1>(*info)[qIds[i]];
			UId tId = std::get<2>(*info)[tIds[i]];
			nodeMap[qId] = tId;
		}
		std::get<0>(*info).push_back(nodeMap);

		// Return false to continue if there are more matches to find.
		// Return true to stop if we have found enough matches.
		return (numMatches >= maxNumMatches);
	}

	template <class State>
	static IsomorphyMap mainIsomorphism(Graph& query, Graph& target, unsigned int maxNumMatches, const time_duration maxDuration = pos_infin, std::function<bool (NodePtr, NodePtr)> nodeCompare = NULL, std::function<bool (EdgePtr, EdgePtr)> edgeCompare = NULL, bool doMerge = true) {
		Graph* q = &query;
		Graph* t = &target;

		GraphPtr qp, tp;

		if (doMerge) {
			qp = Sample<Graph>::mergeMultiEdges(query);
			tp = Sample<Graph>::mergeMultiEdges(target);
			q = qp.get();
			t = tp.get();
		}

		// remember IDs
		// boost bimap fails massively by defining its own _1
		// so we have to do it the ugly way
		IdVec qIds;
		IdVec tIds;

		// init vflib
		VF::ARGEdit qGraphEdit, tGraphEdit;

		// copy query graph nodes
		NodeList nList = q->getNodes();
		for (typename NodeList::iterator it = nList.begin(); it != nList.end(); ++it) {
			qIds.push_back((*it)->getId());
			qGraphEdit.InsertNode(*it);
		}
		// copy query graph edges
		EdgeList eList = q->getEdges();
		for (typename EdgeList::iterator it = eList.begin(); it != eList.end(); ++it) {
			NodePtr sN = (*it)->getSourceNode();
			NodePtr tN = (*it)->getTargetNode();
			optional<UId> sNewId = none;
			optional<UId> tNewId = none;
			for (unsigned int id = 0; id < qIds.size(); ++id) {
				if (qIds[id] == sN->getId()) sNewId = id;
				if (qIds[id] == tN->getId()) tNewId = id;
			}

			if (sNewId && tNewId) {
				qGraphEdit.InsertEdge(sNewId.get(), tNewId.get(), *it);
				qGraphEdit.InsertEdge(tNewId.get(), sNewId.get(), *it);
			}
		}

		// copy target graph nodes
		nList = t->getNodes();
		for (typename NodeList::iterator it = nList.begin(); it != nList.end(); ++it) {
			tIds.push_back((*it)->getId());
			tGraphEdit.InsertNode(*it);
		}
		// copy target graph edges
		eList = t->getEdges();
		for (typename EdgeList::iterator it = eList.begin(); it != eList.end(); ++it) {
			NodePtr sN = (*it)->getSourceNode();
			NodePtr tN = (*it)->getTargetNode();
			optional<UId> sNewId = none;
			optional<UId> tNewId = none;
			for (unsigned int id = 0; id < tIds.size(); ++id) {
				if (tIds[id] == sN->getId()) sNewId = id;
				if (tIds[id] == tN->getId()) tNewId = id;
			}

			if (sNewId && tNewId) {
				tGraphEdit.InsertEdge(sNewId.get(), tNewId.get(), *it);
				tGraphEdit.InsertEdge(tNewId.get(), sNewId.get(), *it);
			}
		}

		// VFlib uses this to do static type checks
		VF::ARGraph< Node, Edge > qVFGraph(&qGraphEdit);
		VF::ARGraph< Node, Edge > tVFGraph(&tGraphEdit);

		// feed the machine...
		Comparator<NodePtr>* nodeComparator = new Comparator<NodePtr>(nodeCompare, maxDuration);
		Comparator<EdgePtr>* edgeComparator = new Comparator<EdgePtr>(edgeCompare, maxDuration);

		qVFGraph.SetNodeComparator(nodeComparator);
		qVFGraph.SetEdgeComparator(edgeComparator);
		State s0(&qVFGraph, &tVFGraph);

		// ...and push the button
		IsomorphyMap resultMap;
		unsigned int numMatches = 0;
		VisitorData data(resultMap, qIds, tIds, numMatches, maxNumMatches);

		nodeComparator->resetStartTime();
		edgeComparator->resetStartTime();
		VF::match(&s0, &matchVisitor, static_cast<void*>(&data));

		return resultMap;
	}


};

} // Algorithm
} // Graph

#endif /* MATCH_H_ */
