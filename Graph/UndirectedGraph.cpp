#include "UndirectedGraph.h"

#include <cmath>
using std::min;

#include <expat.h>

#include <cstdio>
#include <cstdlib>
#include <ctime>

namespace Graph {

// GRAPH //
UndirectedGraph::~UndirectedGraph() {
	for (list<EdgePtr>::iterator it = m_edges.begin(); it != m_edges.end(); ++it) {
		delete *it;
	}
	for (list<NodePtr>::iterator it = m_nodes.begin(); it != m_nodes.end(); ++it) {
		delete *it;
	}
	m_edges.clear();
	m_nodes.clear();
}

void UndirectedGraph::removeNode(UId id) {
	EdgeList::iterator eIt = m_edges.begin();
	while (eIt != m_edges.end()) {
		if ((*eIt)->getSourceNode()->getId() == id || (*eIt)->getTargetNode()->getId() == id) {
			(*eIt)->getSourceNode()->removeEdge((*eIt)->getId());
			(*eIt)->getTargetNode()->removeEdge((*eIt)->getId());
			delete *eIt;
			eIt = m_edges.erase(eIt);
			continue;
		}
		++eIt;
	}

	NodeList::iterator nIt = m_nodes.begin();
	while (nIt != m_nodes.end()) {
		if ((*nIt)->getId() == id) {
			delete *nIt;
			nIt = m_nodes.erase(nIt);
			continue;
		}
		++nIt;
	}
}

void UndirectedGraph::removeEdge(UId id) {
	EdgeList::iterator eIt = m_edges.begin();
	while (eIt != m_edges.end()) {
		if ((*eIt)->getId() == id) {
			(*eIt)->getSourceNode()->removeEdge(id);
			(*eIt)->getTargetNode()->removeEdge(id);
			delete *eIt;
			eIt = m_edges.erase(eIt);
			continue;
		}
		++eIt;
	}
}

void UndirectedGraph::serializeDOT(string name, const char* filename, std::function<string (NodePtr)> getNodeAttributes, std::function<string (EdgePtr)> getEdgeAttributes, const Indentation& indent) {
	ofstream out(filename);
	if (!out.good()) return;

	serializeDOT(name, out, getNodeAttributes, getEdgeAttributes, indent);
	out.close();
}

void UndirectedGraph::serializeDOT(string name, ofstream& out, std::function<string (NodePtr)> getNodeAttributes, std::function<string (EdgePtr)> getEdgeAttributes, const Indentation& indent) {
	string ind = indent.getIndentation();
	out << ind << "strict graph " << name << " {\n";
	Indentation nodeIndent(indent);
	nodeIndent.increaseDepth();

	for (list<NodePtr>::iterator nIt = m_nodes.begin(); nIt != m_nodes.end(); ++nIt) {
		out << nodeIndent.getIndentation() << (*nIt)->m_id;
		if(getNodeAttributes) out << "[" << getNodeAttributes(*nIt) << "]";
		out << ";\n";
	}
	out << "\n";
	for (list<EdgePtr>::iterator eIt = m_edges.begin(); eIt != m_edges.end(); ++eIt) {
		out << nodeIndent.getIndentation() << (*eIt)->getSourceNode()->m_id << " -- " << (*eIt)->getTargetNode()->m_id;
		if (getEdgeAttributes) out	<< "[" << getEdgeAttributes(*eIt) << "]";
		out << ";\n";
	}
	out << ind << "}\n";
}

void UndirectedGraph::serializeGraphML(string filename, const Indentation& indent) {
	std::ofstream out( filename.c_str() );
	if ( !out.good() ) {
		return;
	}

	string content;
	serializeGraphMLToString(content, indent);
	out << content;

	out.close();
}

void UndirectedGraph::serializeGraphMLToString(string& content, const Indentation& indent) {
	content = "";

	map<string, std::tuple<string, string, string> > attrMap;
	Indentation elemIndent = indent;
	elemIndent.increaseDepth();
	stringstream elements;

	elements << elemIndent.getIndentation() << "<graph id=\"G\" edgedefault=\"undirected\" parse.nodeids=\"canonical\" parse.edgeids=\"canonical\" parse.order=\"nodesfirst\">\n";
	elemIndent.increaseDepth();

	string innerInd = elemIndent.getIndentation();
	for (map<string, any>::iterator it = m_attributes.begin(); it != m_attributes.end(); ++it) {
		optional< pair<string, string> > a = getTypedAttributeStr(it->first);
		if (!a) continue;
		map<string, std::tuple<string, string, string> >::iterator findIt = attrMap.find(it->first);
		if (findIt == attrMap.end()) {
			stringstream vStr; vStr << "key" << attrMap.size();
			attrMap[it->first] = std::make_tuple(vStr.str(), string("graph"), a.get().second);
			elements << innerInd << "<data key=\"" << vStr.str() << "\">" << a.get().first << "</data>\n";
		} else {
			elements << innerInd << "<data key=\"" << std::get<0>(findIt->second) << "\">" << a.get().first << "</data>\n";
		}
	}

	for (list<NodePtr>::iterator nIt = m_nodes.begin(); nIt != m_nodes.end(); ++nIt) {
		serializeGraphMLNode(elements, *nIt, attrMap, elemIndent);
	}
	for (list<EdgePtr>::iterator eIt = m_edges.begin(); eIt != m_edges.end(); ++eIt) {
		serializeGraphMLEdge(elements, *eIt, attrMap, elemIndent);
	}

	elemIndent.decreaseDepth();
	elements << elemIndent.getIndentation() << "</graph>\n";

	std::stringstream result;

	result << indent.getIndentation() << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
	    << indent.getIndentation() << "<graphml xmlns=\"http://graphml.graphdrawing.org/xmlns\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd\">\n";

	for (map<string, std::tuple<string, string, string> >::iterator it = attrMap.begin(); it != attrMap.end(); ++it) {
		result << elemIndent.getIndentation()
		<< "<key id=\"" << std::get<0>(it->second)
		<< "\" for=\"" << std::get<1>(it->second)
		<< "\" attr.name=\"" << it->first
		<< "\" attr.type=\"" << std::get<2>(it->second) << "\" />\n";
	}

	result << elements.str();

	result << "</graphml>\n";

	content = result.str();
}

void UndirectedGraph::deserializeGraphML(string filename) {
	// create input file stream and write it linewise into string content
	std::ifstream in( filename.c_str() );
	if ( !in.good() ) {
		return;
	}
	std::string content, line;
	while (getline(in, line)) { content += line + "\n"; }
	in.close();
	deserializeGraphMLFromString(content);

	//// from here on we solely work on content
	//XML_Parser p = XML_ParserCreate(NULL);
	//if (!p) return;

	//ParseInfo* parseInfo = new ParseInfo(this);

	//XML_SetUserData(p, parseInfo);
	//XML_SetElementHandler(p, &UndirectedGraph::deserializeGraphMLStartTag, &UndirectedGraph::deserializeGraphMLEndTag);
	//XML_SetCharacterDataHandler(p, &UndirectedGraph::deserializeGraphMLCharData);
	//XML_Parse( p, content.c_str(), content.length(), false );

	//// foreach edgename --> (sourcename, targetname) set edgeMap[edgename] = nodeptr(sourcename)->linkTo(targetname)
	//for (map< string, std::tuple<string,string> >::iterator it = parseInfo->edges.begin(); it != parseInfo->edges.end(); ++it) {
	//	EdgePtr edge = parseInfo->nodeMap[std::get<0>(it->second)]->linkTo(parseInfo->nodeMap[std::get<1>(it->second)]);
	//	if (edge) {
	//		parseInfo->edgeMap[it->first] = edge;
	//	}
	//}
	//// foreach (dataid, value) set graph[settingname(dataid)] = value
	//for (vector< std::tuple<string,string> >::iterator it = parseInfo->graphData.begin(); it != parseInfo->graphData.end(); ++it) {
	//	if (std::get<0>(parseInfo->keyMap[std::get<0>(*it)]) != "graph") continue;
	//	addTypedAttribute(std::get<2>(parseInfo->keyMap[std::get<0>(*it)]), std::get<1>(parseInfo->keyMap[std::get<0>(*it)]), std::get<1>(*it));
	//}
	//// foreach nodename --> (dataid, value) set nodeptr(nodename)->[settingname(dataid)] = value
	//for (map< string, vector< std::tuple<string,string> > >::iterator it = parseInfo->nodeDataMap.begin(); it != parseInfo->nodeDataMap.end(); ++it) {
	//	for (vector< std::tuple<string,string> >::iterator dit = it->second.begin(); dit != it->second.end(); ++dit) {
	//		if (std::get<0>(parseInfo->keyMap[std::get<0>(*dit)]) != "node") continue;
	//		NodePtr n = parseInfo->nodeMap[it->first];
	//		n->addTypedAttribute(std::get<2>(parseInfo->keyMap[std::get<0>(*dit)]), std::get<1>(parseInfo->keyMap[std::get<0>(*dit)]), std::get<1>(*dit));
	//	}
	//}
	//// foreach edgename --> (dataid, value) set edgeptr(edgename)->[settingname(dataid)] = value
	//for (map< string, vector< std::tuple<string,string> > >::iterator it = parseInfo->edgeDataMap.begin(); it != parseInfo->edgeDataMap.end(); ++it) {
	//	if (parseInfo->edgeMap.find(it->first) == parseInfo->edgeMap.end()) continue;
	//	for (vector< std::tuple<string,string> >::iterator dit = it->second.begin(); dit != it->second.end(); ++dit) {
	//		if (std::get<0>(parseInfo->keyMap[std::get<0>(*dit)]) != "edge") continue;

	//		EdgePtr e = parseInfo->edgeMap[it->first];
	//		e->addTypedAttribute(std::get<2>(parseInfo->keyMap[std::get<0>(*dit)]), std::get<1>(parseInfo->keyMap[std::get<0>(*dit)]), std::get<1>(*dit));
	//	}
	//}

	//XML_ParserFree(p);
	//delete parseInfo;
}

void UndirectedGraph::deserializeGraphMLFromString(const std::string graphMLString) {
		// from here on we solely work on content
	XML_Parser p = XML_ParserCreate(NULL);
	if (!p) return;

	ParseInfo* parseInfo = new ParseInfo(this);

	XML_SetUserData(p, parseInfo);
	XML_SetElementHandler(p, &UndirectedGraph::deserializeGraphMLStartTag, &UndirectedGraph::deserializeGraphMLEndTag);
	XML_SetCharacterDataHandler(p, &UndirectedGraph::deserializeGraphMLCharData);
	XML_Parse( p, graphMLString.c_str(), graphMLString.length(), false );

	// foreach edgename --> (sourcename, targetname) set edgeMap[edgename] = nodeptr(sourcename)->linkTo(targetname)
	for (map< string, std::tuple<string,string> >::iterator it = parseInfo->edges.begin(); it != parseInfo->edges.end(); ++it) {
		EdgePtr edge = parseInfo->nodeMap[std::get<0>(it->second)]->linkTo(parseInfo->nodeMap[std::get<1>(it->second)]);
		if (edge) {
			parseInfo->edgeMap[it->first] = edge;
		}
	}
	// foreach (dataid, value) set graph[settingname(dataid)] = value
	for (vector< std::tuple<string,string> >::iterator it = parseInfo->graphData.begin(); it != parseInfo->graphData.end(); ++it) {
		if (std::get<0>(parseInfo->keyMap[std::get<0>(*it)]) != "graph") continue;
		addTypedAttribute(std::get<2>(parseInfo->keyMap[std::get<0>(*it)]), std::get<1>(parseInfo->keyMap[std::get<0>(*it)]), std::get<1>(*it));
	}
	// foreach nodename --> (dataid, value) set nodeptr(nodename)->[settingname(dataid)] = value
	for (map< string, vector< std::tuple<string,string> > >::iterator it = parseInfo->nodeDataMap.begin(); it != parseInfo->nodeDataMap.end(); ++it) {
		for (vector< std::tuple<string,string> >::iterator dit = it->second.begin(); dit != it->second.end(); ++dit) {
			if (std::get<0>(parseInfo->keyMap[std::get<0>(*dit)]) != "node") continue;
			NodePtr n = parseInfo->nodeMap[it->first];
			n->addTypedAttribute(std::get<2>(parseInfo->keyMap[std::get<0>(*dit)]), std::get<1>(parseInfo->keyMap[std::get<0>(*dit)]), std::get<1>(*dit));
		}
	}
	// foreach edgename --> (dataid, value) set edgeptr(edgename)->[settingname(dataid)] = value
	for (map< string, vector< std::tuple<string,string> > >::iterator it = parseInfo->edgeDataMap.begin(); it != parseInfo->edgeDataMap.end(); ++it) {
		if (parseInfo->edgeMap.find(it->first) == parseInfo->edgeMap.end()) continue;
		for (vector< std::tuple<string,string> >::iterator dit = it->second.begin(); dit != it->second.end(); ++dit) {
			if (std::get<0>(parseInfo->keyMap[std::get<0>(*dit)]) != "edge") continue;

			EdgePtr e = parseInfo->edgeMap[it->first];
			e->addTypedAttribute(std::get<2>(parseInfo->keyMap[std::get<0>(*dit)]), std::get<1>(parseInfo->keyMap[std::get<0>(*dit)]), std::get<1>(*dit));
		}
	}

	XML_ParserFree(p);
	delete parseInfo;

}

void UndirectedGraph::deserialize(string filename) {
	deserializeGraphML(filename);
}

void UndirectedGraph::deserializeGraphMLStartTag(void *data, const char* element, const char** attributes) {
	ParseInfo* parseInfo = static_cast<ParseInfo*>(data);

	if (!strcmp(element, "node")) {
		parseInfo->currentEntity = NODE;
		NodePtr node = parseInfo->graph->addNode();

		std::string id;
		for (int i = 0; attributes[i]; i += 2) {
			if (!strcmp(attributes[i], "id")) {
				id = attributes[i+1];
			}
		}
		parseInfo->nodeMap[id] = node;
		parseInfo->currentElementKey = id;
	}

	if (!strcmp(element, "edge")) {
		parseInfo->currentEntity = EDGE;
		string sourceNode, targetNode, id;
		for (int i = 0; attributes[i]; i += 2) {
			if (!strcmp(attributes[i], "id")) {
				id = attributes[i+1];
			}
			if (!strcmp(attributes[i], "source")) {
				sourceNode = attributes[i+1];
			}
			if (!strcmp(attributes[i], "target")) {
				targetNode = attributes[i+1];
			}
		}
		parseInfo->edges[id] = std::make_tuple(sourceNode, targetNode);
		parseInfo->currentElementKey = id;
	}

	if (!strcmp(element, "key") && parseInfo->currentEntity == GRAPH) {
		std::string id = "";
		std::string target = "";
		std::string type = "string";
		std::string name = "";
		for (int i = 0; attributes[i]; i += 2) {
			if (!strcmp(attributes[i], "id")) {
				id = attributes[i+1];
			} else if (!strcmp(attributes[i], "for")) {
				target = attributes[i+1];
			} else if (!strcmp(attributes[i], "attr.name")) {
				name = attributes[i+1];
			} else if (!strcmp(attributes[i], "attr.type")) {
				type = attributes[i+1];
			}
		}
		if (id == "" || target == "" || name == "") return;

		parseInfo->keyMap[id] = std::make_tuple(target, type, name);
	}

	if (!strcmp(element, "data")) {
		for (int i = 0; attributes[i]; i += 2) {
			if (!strcmp(attributes[i], "key")) {
				parseInfo->currentDataKey = attributes[i+1];
				parseInfo->charData.clear();
			}
		}
	}
}

void UndirectedGraph::deserializeGraphMLEndTag(void *data, const char* element) {
	ParseInfo* parseInfo = static_cast<ParseInfo*>(data);
	if (!strcmp(element, "node") || !strcmp(element, "edge")) {
		parseInfo->currentEntity = GRAPH;
	}

	if (!strcmp(element, "data")) {
		switch (parseInfo->currentEntity) {
			case NODE:
				if (parseInfo->nodeDataMap.find(parseInfo->currentElementKey) == parseInfo->nodeDataMap.end()) {
					vector< std::tuple<string,string> > data;
					parseInfo->nodeDataMap[parseInfo->currentElementKey] = data;
				}
				parseInfo->nodeDataMap[parseInfo->currentElementKey].push_back(std::make_tuple(parseInfo->currentDataKey, string(parseInfo->charData)));
				break;
			case EDGE:
				if (parseInfo->edgeDataMap.find(parseInfo->currentElementKey) == parseInfo->edgeDataMap.end()) {
					vector< std::tuple<string,string> > data;
					parseInfo->edgeDataMap[parseInfo->currentElementKey] = data;
				}
				parseInfo->edgeDataMap[parseInfo->currentElementKey].push_back(std::make_tuple(parseInfo->currentDataKey, string(parseInfo->charData)));
				break;
			default:
				parseInfo->graphData.push_back( std::make_tuple(parseInfo->currentDataKey, string(parseInfo->charData)) );
		}
	}
}

void UndirectedGraph::deserializeGraphMLCharData(void *data, const char* cdata, int len) {
	ParseInfo* parseInfo = static_cast<ParseInfo*>(data);
	parseInfo->charData += std::string(cdata, len);
}

void UndirectedGraph::serializeGraphMLNode(stringstream& out, NodePtr n, map<string, std::tuple<string, string, string> >& attrMap, Indentation indent) {
	string outerInd = indent.getIndentation();
	Indentation inIndent = indent;
	inIndent.increaseDepth();
	string innerInd = inIndent.getIndentation();
	out << outerInd << "<node id=\"n" << n->getId() << "\">\n";
	map<string, any> attrs = n->getAttributes();
	for (map<string, any>::iterator it = attrs.begin(); it != attrs.end(); ++it) {
		optional< pair<string, string> > a = n->getTypedAttributeStr(it->first);
		if (!a) continue;
		map<string, std::tuple<string, string, string> >::iterator findIt = attrMap.find(it->first);
		if (findIt == attrMap.end()) {
			stringstream vStr; vStr << "key" << attrMap.size();
			attrMap[it->first] = std::make_tuple(vStr.str(), string("node"), a.get().second);
			out << innerInd << "<data key=\"" << vStr.str() << "\">" << a.get().first << "</data>\n";
		} else {
			out << innerInd << "<data key=\"" << std::get<0>(findIt->second) << "\">" << a.get().first << "</data>\n";
		}
	}
	out << outerInd << "</node>\n";
}

void UndirectedGraph::serializeGraphMLEdge(stringstream& out, EdgePtr e, map<string, std::tuple<string, string, string> >& attrMap, Indentation indent) {
	string outerInd = indent.getIndentation();
	Indentation inIndent = indent;
	inIndent.increaseDepth();
	string innerInd = inIndent.getIndentation();
	out << outerInd << "<edge id=\"e" << e->getId()
	    << "\" source=\"n" << e->getSourceNode()->getId()
	    << "\" target=\"n" << e->getTargetNode()->getId() << "\">\n";
	map<string, any> attrs = e->getAttributes();
	for (map<string, any>::iterator it = attrs.begin(); it != attrs.end(); ++it) {
		optional< pair<string, string> > a = e->getTypedAttributeStr(it->first);
		if (!a) continue;
		map<string, std::tuple<string, string, string> >::iterator findIt = attrMap.find(it->first);
		if (findIt == attrMap.end()) {
			stringstream vStr; vStr << "key" << attrMap.size();
			attrMap[it->first] = std::make_tuple(vStr.str(), string("edge"), a.get().second);
			out << innerInd << "<data key=\"" << vStr.str() << "\">" << a.get().first << "</data>\n";
		} else {
			out << innerInd << "<data key=\"" << std::get<0>(findIt->second) << "\">" << a.get().first << "</data>\n";
		}
	}
	out << outerInd << "</edge>\n";
}

#ifdef USE_LAPACKPP
UndirectedGraph::LaplacianMatrix UndirectedGraph::getLaplacianMatrix(bool normalize) {
	resetNodeIds();
	unsigned int numNodes = getNodeCount();
	LaplacianMatrix result = LaplacianMatrix::zeros(numNodes, numNodes);

	for (NodeList::const_iterator it = m_nodes.begin(); it != m_nodes.end(); ++it) {
		UId index = (*it)->getId();

		set<NodePtr> neighbors = (*it)->getNeighbors();
		result(index,index) = normalize ? !!neighbors.size() : neighbors.size();

		for (set<NodePtr>::iterator nIt = neighbors.begin(); nIt != neighbors.end(); ++nIt) {
			UId nId = (*nIt)->getId();
			result(index,nId) = result(nId,index) = -1.0;
			if (normalize) {
				double normDivisor = sqrt(static_cast<double>(neighbors.size() * (*nIt)->getNeighbors().size()));
				result(index,nId) = result(nId,index) /= normDivisor;
			}
		}
	}

	return result;
}
#endif

// NODE //
UndirectedGraph::Node::~Node() {
	/* do not destroy edges here as graph class destructor handles these */
	m_edges.clear();
}





}
