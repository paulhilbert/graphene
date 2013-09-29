#ifndef RCG_H_
#define RCG_H_

#include <vector>
#include <algorithm>
using std::vector;
using std::reverse;

#include <memory>

#include <boost/optional.hpp>
using boost::optional;
#include <boost/none.hpp>
using boost::none;
#include <boost/lexical_cast.hpp>
using boost::lexical_cast;

#include "UndirectedGraph.h"

#include <Math/Vector.h>
#include <Math/Trigonometry.h>
using namespace Math;

/* #include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef CGAL::Polygon_2<K> Polygon; */

#include <StringUtils/VectorString.h>
#include <StringUtils/SplitString.h>
using namespace StringUtils;


namespace Graph {

class RCG : public UndirectedGraph {
	public:
		typedef RCG               Graph;
		typedef std::shared_ptr<Graph> GraphPtr;

		static const unsigned int CLEANUP_MAKE_BALCONIES = 1;
		static const unsigned int CLEANUP_DELETE_WINDOWS = 2;
		static const unsigned int CLEANUP_COMPUTE_HAS_WINDOWS = 4;

	public:
		RCG() : UndirectedGraph() { }
		virtual ~RCG() {}

		virtual void serializeGraphML(string filename, const Indentation& indent = Indentation()) {
			std::ofstream out( filename.c_str() );
			if ( !out.good() ) {
				return;
			}

			string content;
			serializeGraphMLToString(content, indent);
			out << content;

			out.close();
		}
		virtual void serializeGraphMLToString(string& content, const Indentation& indent = Indentation()) {
			// We need to remember the old attributes in vector
			// representation to restore them after serialization.
			vector< optional< Vector<float, 3> > >           oldPositionList;
			vector< optional< vector< Vector<float, 3> > > > oldRoomVerticesList;
			vector< optional< Vector<float, 3> > >           oldOpeningDirectionList;
			vector< optional< Vector<float, 3> > >           oldOpeningPositionList;
			vector< optional< Vector<float, 3> > >           oldNodeColorList;
			vector< optional< Vector<float, 3> > >           oldEdgeColorList;
			vector< optional< vector<float> > >              oldNodeDescList;

			NodeList& nList = getNodes();
			for (NodeList::iterator it = nList.begin(); it != nList.end(); ++it) {
				optional< Vector<float, 3> >           oldPosition     = none;
				optional< vector< Vector<float, 3> > > oldRoomVertices = none;
				optional< Vector<float, 3> >           oldNodeColor    = none;
				optional< vector<float> >              oldNodeDesc     = none;

				try {
					Vector<float, 3> vec = any_cast< Vector<float, 3> >( (**it)["Position"] );
					oldPosition = vec;
					(**it)["Position"] = Vec2Str(vec);
				} catch (...) { /* ignore */ }
				try {
					vector< Vector<float, 3> > vecs = any_cast< vector< Vector<float, 3> > >( (**it)["RoomVertices"] );
					oldRoomVertices = vecs;
					(**it)["RoomVertices"] = Vecs2Str(vecs);
				} catch (...) { /* ignore */ }
				try {
					Vector<float, 3> vec = any_cast< Vector<float, 3> >( (**it)["Color"] );
					oldNodeColor = vec;
					(**it)["Color"] = Vec2Str(vec);
				} catch (...) { /* ignore */ }
				try {
					vector<float> vec = any_cast< vector<float> >( (**it)["NodeDesc"] );
					oldNodeDesc = vec;
					string descStr;
					for (unsigned int i = 0; i < vec.size(); ++i) {
						descStr += lexical_cast<string>(vec[i]);
						if (i != vec.size()-1) descStr += ",";
					}
					(**it)["NodeDesc"] = descStr;
				} catch (...) { /* ignore */ }

				oldPositionList.push_back(oldPosition);
				oldRoomVerticesList.push_back(oldRoomVertices);
				oldNodeColorList.push_back(oldNodeColor);
				oldNodeDescList.push_back(oldNodeDesc);
			}

			EdgeList& eList = getEdges();
			for (EdgeList::iterator it = eList.begin(); it != eList.end(); ++it) {
				optional< Vector<float, 3> > oldOpeningDirection = none;
				optional< Vector<float, 3> > oldOpeningPosition  = none;
				optional< Vector<float, 3> > oldEdgeColor        = none;

				try {
					Vector<float, 3> dir = any_cast< Vector<float, 3> >( (**it)["OpeningDirection"] );
					oldOpeningDirection = dir;
					(**it)["OpeningDirection"] = Vec2Str(dir);
				} catch (...) { /* ignore */ }
				try {
					Vector<float, 3> pos = any_cast< Vector<float, 3> >( (**it)["OpeningPosition"] );
					oldOpeningPosition = pos;
					(**it)["OpeningPosition"] = Vec2Str(pos);
				} catch (...) { /* ignore */ }
				try {
					Vector<float, 3> vec = any_cast< Vector<float, 3> >( (**it)["EdgeColor"] );
					oldEdgeColor = vec;
					(**it)["EdgeColor"] = Vec2Str(vec);
				} catch (...) { /* ignore */ }

				oldOpeningDirectionList.push_back(oldOpeningDirection);
				oldOpeningPositionList.push_back(oldOpeningPosition);
				oldEdgeColorList.push_back(oldEdgeColor);
			}


			UndirectedGraph::serializeGraphMLToString(content, indent);


			// Restore the old vector attributes.
			unsigned int listIndex = 0;
			for (NodeList::iterator it = nList.begin(); it != nList.end(); ++it, ++listIndex) {
				if (oldPositionList[listIndex])     (**it)["Position"]     = oldPositionList[listIndex].get();
				if (oldRoomVerticesList[listIndex]) (**it)["RoomVertices"] = oldRoomVerticesList[listIndex].get();
				if (oldNodeColorList[listIndex])    (**it)["Color"]        = oldNodeColorList[listIndex].get();
				if (oldNodeDescList[listIndex])     (**it)["NodeDesc"]     = oldNodeDescList[listIndex].get();
			}

			listIndex = 0;
			for (EdgeList::iterator it = eList.begin(); it != eList.end(); ++it, ++listIndex) {
				if (oldOpeningDirectionList[listIndex]) (**it)["OpeningDirection"] = oldOpeningDirectionList[listIndex].get();
				if (oldOpeningPositionList[listIndex])  (**it)["OpeningPosition"]  = oldOpeningPositionList[listIndex].get();
				if (oldEdgeColorList[listIndex])        (**it)["EdgeColor"]        = oldEdgeColorList[listIndex].get();
			}
		}
		virtual void PreCleanup(unsigned int cleanupOptions = 0)
		{
			NodeList nList = getNodes();
			for (NodeList::iterator it = nList.begin(); it != nList.end(); ++it) {
				optional<string> posString = (*it)->get<string>("Position");
				if (posString) {
					Vector<float, 3> pos = Str2Vec<float,3>(posString.get());
					(**it)["Position"] = pos;
				}
				optional<string> vecsString = (*it)->get<string>("RoomVertices");
				if (vecsString) {
					vector< Vector<float, 3> > vecs = Str2Vecs<float,3>(vecsString.get());
					/*Polygon poly;
					for (unsigned int i=0; i<vecs.size(); ++i) {
						poly.push_back( Point(vecs[i][0], vecs[i][1]) );
					}
					if (poly.size() && poly.is_simple() && poly.orientation() == CGAL::CLOCKWISE) reverse(vecs.begin(), vecs.end()); */
					(**it)["RoomVertices"] = vecs;
				}
				optional<string> colorString = (*it)->get<string>("Color");
				if (colorString) {
					Vector<float, 3> color = Str2Vec<float,3>(colorString.get());
					(**it)["Color"] = color;
				}
				optional<string> nodeDescString = (*it)->get<string>("NodeDesc");
				if (nodeDescString) {
					vector<float> nodeDescVector;
					vector<string> nodeDescTokens = splitString(nodeDescString.get(), ",");
					for (unsigned int i = 0; i < nodeDescTokens.size(); ++i) {
						nodeDescVector.push_back(lexical_cast<float>(nodeDescTokens[i]));
					}
					(**it)["NodeDesc"] = nodeDescVector;
				}
			}

			EdgeList eList = getEdges();
			for (EdgeList::iterator it = eList.begin(); it != eList.end(); ++it) {
				optional<string> dirString = (*it)->get<string>("OpeningDirection");
				if (dirString) {
					Vector<float, 3> dir = Str2Vec<float,3>(dirString.get());
					(**it)["OpeningDirection"] = dir;
				}
				optional<string> posString = (*it)->get<string>("OpeningPosition");
				if (posString) {
					Vector<float, 3> pos = Str2Vec<float,3>(posString.get());
					(**it)["OpeningPosition"] = pos;
				}
				optional<string> colorString = (*it)->get<string>("EdgeColor");
				if (colorString) {
					Vector<float, 3> color = Str2Vec<float,3>(colorString.get());
					(**it)["EdgeColor"] = color;
				}
			}

			if (cleanupOptions) {
				cleanup(cleanupOptions);
			}
		}
		virtual void deserializeGraphMLFromString(const std::string graphMLString, unsigned int cleanupOptions = 0) {
			UndirectedGraph::deserializeGraphMLFromString(graphMLString);
			PreCleanup(cleanupOptions);
		}

		virtual void deserializeGraphML(string filename, unsigned int cleanupOptions = 0) {
			UndirectedGraph::deserializeGraphML(filename);
			PreCleanup(cleanupOptions);
			/*NodeList nList = getNodes();
			for (NodeList::iterator it = nList.begin(); it != nList.end(); ++it) {
				optional<string> posString = (*it)->get<string>("Position");
				if (posString) {
					Vector<float, 3> pos = Str2Vec<float,3>(posString.get());
					(**it)["Position"] = pos;
				}
				optional<string> vecsString = (*it)->get<string>("RoomVertices");
				if (vecsString) {
					vector< Vector<float, 3> > vecs = Str2Vecs<float,3>(vecsString.get());
					Polygon poly;
					for (unsigned int i=0; i<vecs.size(); ++i) {
						poly.push_back( Point(vecs[i][0], vecs[i][1]) );
					}
					if (poly.size() && poly.is_simple() && poly.orientation() == CGAL::CLOCKWISE) reverse(vecs.begin(), vecs.end());
					(**it)["RoomVertices"] = vecs;
				}
				optional<string> colorString = (*it)->get<string>("Color");
				if (colorString) {
					Vector<float, 3> color = Str2Vec<float,3>(colorString.get());
					(**it)["Color"] = color;
				}
				optional<string> nodeDescString = (*it)->get<string>("NodeDesc");
				if (nodeDescString) {
					vector<float> nodeDescVector;
					vector<string> nodeDescTokens = splitString(nodeDescString.get(), ",");
					for (unsigned int i = 0; i < nodeDescTokens.size(); ++i) {
						nodeDescVector.push_back(lexical_cast<float>(nodeDescTokens[i]));
					}
					(**it)["NodeDesc"] = nodeDescVector;
				}
			}

			EdgeList eList = getEdges();
			for (EdgeList::iterator it = eList.begin(); it != eList.end(); ++it) {
				optional<string> dirString = (*it)->get<string>("OpeningDirection");
				if (dirString) {
					Vector<float, 3> dir = Str2Vec<float,3>(dirString.get());
					(**it)["OpeningDirection"] = dir;
				}
				optional<string> posString = (*it)->get<string>("OpeningPosition");
				if (posString) {
					Vector<float, 3> pos = Str2Vec<float,3>(posString.get());
					(**it)["OpeningPosition"] = pos;
				}
				optional<string> colorString = (*it)->get<string>("EdgeColor");
				if (colorString) {
					Vector<float, 3> color = Str2Vec<float,3>(colorString.get());
					(**it)["EdgeColor"] = color;
				}
			}

			if (cleanupOptions) {
				cleanup(cleanupOptions);
			}*/
		}

		virtual void cleanup(unsigned int options = CLEANUP_MAKE_BALCONIES) {
			// ROOM = 0, OUTSIDE = 1, BALCONY = 2
			// DOOR = 0, WINDOW = 1, STAIRS = 4

			bool optionMakeBalconies = (options & CLEANUP_MAKE_BALCONIES);
			bool optionDeleteWindows = (options & CLEANUP_DELETE_WINDOWS);
			bool optionComputeHasWindows = (options & CLEANUP_COMPUTE_HAS_WINDOWS);

			NodeList nList = getNodes();

			float roomAreaSum = 0.f;
			for (NodeList::iterator it = nList.begin(); it != nList.end(); ++it) {
				Node& node = **it;

				bool hasRoomVertices = false;

				// Find out whether this room has room vertices.
				optional< vector< Vector<float, 3> > > roomVertices = node.get< vector< Vector<float, 3> > >("RoomVertices");

				// Compute and store room area.
				if (roomVertices) {
					hasRoomVertices = roomVertices.get().size() > 0;
					if (hasRoomVertices) {
						float area = gaussianTrapezEquation(roomVertices.get());
						if (area <= 0.f) {
							std::cout << "Warning: Room with area <= 0 (" << area << ")" << std::endl;
						}
						node["RoomArea"] = area;
						roomAreaSum += area;
					}
				}

				optional<int> roomType = node.get<int>("RoomType");
				if (roomType) {
					// Warn about rooms without room vertices.
					if (roomType.get() == 0 && !hasRoomVertices) {
						//std::cerr << "Warning: Room without room vertices!" << std::endl;
					}

					// Make balconies out of outside nodes with room vertices (if desired).
					if (optionMakeBalconies && roomType.get() == 1 && hasRoomVertices) {
						node["RoomType"] = 2;
					}
				}
			}

			if (!get<float>("TotalArea")) (*this)["TotalArea"] = roomAreaSum;

/*
			// Normalize room areas
			nList = getNodes();
			for (NodeList::iterator it = nList.begin(); it != nList.end(); ++it) {
				try {
					any_cast<float&>((**it)["RoomArea"]) /= roomAreaSum;
				} catch (...) {}
			}
*/
			// Combine outside nodes.
			NodePtr outsideNode = NULL; // This will be the ultimate outside node. I mean THE outside node.
			nList = getNodes();
			for (NodeList::iterator it = nList.begin(); it != nList.end(); ++it) {
				Node& node = **it;
				optional<int> roomType = node.get<int>("RoomType");
				if (roomType && roomType.get() == 1) {
					// If we haven't found an outside node yet, use this one.
					if (!outsideNode) {
						outsideNode = &node;
						(*outsideNode)["Position"] = Vector<float, 3>(0, 0, 0);
					// If we already have an outside node, merge.
					} else {
						// Get edge list between this outside node and the rooms.
						const EdgeList& edges = node.getEdges();
						// We need to connect all adjacent rooms to the outside node.
						for (EdgeList::const_iterator eIt = edges.begin(); eIt != edges.end(); ++eIt) {
							// Get the end of the edge which is NOT the outside node...
							NodePtr n = ((*eIt)->getSourceNode() == &node) ? (*eIt)->getTargetNode() : (*eIt)->getSourceNode();
							EdgePtr newEdge = n->linkTo(outsideNode);
							if (newEdge) RCG::combineEdgeAttributes(*eIt, newEdge);
							//if (newEdge) RCG::copyEdgeAttributes(*eIt, newEdge);
						}
						// Remove the redundant outside node.
						node.remove();
					}
				}
			}

			// We're now going to connect balconies to the outside node via windows.
			// However this only makes sense if we're not going to delete windows anyway.
			if (!optionDeleteWindows && optionMakeBalconies) {
				// Add artificial outside node if we don't have one yet.
				if (!outsideNode) {
					std::cout << "WARNING: Creating outside node because we don't have one yet." << std::endl;
					outsideNode = addNode();
					(*outsideNode)["RoomType"] = 1;
					(*outsideNode)["Position"] = Vector<float, 3>(0, 0, 0);
				}

				// Connect balconies to the outside node (via window).
				nList = getNodes();
				for (NodeList::iterator it = nList.begin(); it != nList.end(); ++it) {
					Node& node = **it;
					optional<int> roomType = node.get<int>("RoomType");
					if(roomType && roomType.get() == 2) {
						EdgeList edges = node.getEdges();
						bool alreadyHasEdge = false;
						for (EdgeList::iterator eIt = edges.begin(); eIt != edges.end(); ++eIt) {
							// Don't connect if already connected to the outside node.
							if ((*eIt)->getSourceNode() == outsideNode || (*eIt)->getTargetNode() == outsideNode) {
								alreadyHasEdge = true;
								break;
							}
						}
						if (!alreadyHasEdge) {
							EdgePtr newEdge = node.linkTo(outsideNode);
							(*newEdge)["PassageType"] = 1; // WINDOW
						}
					}
				}
			}

			if (optionComputeHasWindows) {
				nList = getNodes();
				for (NodeList::iterator it = nList.begin(); it != nList.end(); ++it) {
					Node& node = **it;
					// Add hasWindows attribute for prototype matching
					EdgeList& edges = node.getEdges();
					int hasWindows = 0;
					for (EdgeList::iterator it = edges.begin(); it != edges.end(); ++it) {
						if (any_cast<int>((**it)["PassageType"]) == 1) {
							hasWindows = 1;
							break;
						}
					}
					node["HasWindows"] = hasWindows;
				}
			}

			if (optionDeleteWindows) {
				// Delete window edges and precompute insert/remove costs for other passages
				EdgeList edges = getEdges();
				for (RCG::EdgeList::iterator eIt = edges.begin(); eIt != edges.end(); ++eIt) {
					if (any_cast<int>((**eIt)["PassageType"]) == 1) {
						(*eIt)->remove();
					}
				}
			}
		}

		static void copyEdgeAttributes(EdgePtr from, EdgePtr to) {
			// Perform normal copyAttributes operation.
			UndirectedGraph::copyEdgeAttributes(from, to);

			// Assign fresh MergedPassageTypes vector for this edge.
			Vector<int, 3>& passageTypeVec = any_cast< Vector<int, 3>& >((*to)["MergedPassageTypes"] = Vector<int, 3>(static_cast<int>(0)));

			// Add up our own passage type.
			int toPassageType = any_cast<int>((*to)["PassageType"]);
			++passageTypeVec[toPassageType == 4 ? 2 : toPassageType];
		}

		static void combineEdgeAttributes(EdgePtr from, EdgePtr to) {
			Vector<int, 3> passageTypeVec(static_cast<int>(0));
			bool createNewVec = false;

			// Save existing merged passage types vector.
			try {
				passageTypeVec = any_cast< Vector<int, 3> >((*to)["MergedPassageTypes"]);
			} catch (...) {
				createNewVec = true;
			}

			// If there is no vec yet, we probably need to add our own type to the new vector, if we have a type
			if (createNewVec) {
				try {
					int toPassageType = any_cast<int>((*to)["PassageType"]);
					++passageTypeVec[toPassageType == 4 ? 2 : toPassageType];
				} catch (...) {}
			}

			// Perform normal combineAttributes operation.
			UndirectedGraph::combineEdgeAttributes(from, to);

			// Add up the other edge's passage type.
			int fromPassageType = any_cast<int>((*from)["PassageType"]);
			++passageTypeVec[fromPassageType == 4 ? 2 : fromPassageType]; // The index is a mapping from passage type to vector index, 4->2, 1->1, 0->0.

			// Restore modified passage type vec in target edge.
			(*to)["MergedPassageTypes"] = passageTypeVec;
		}
};

}

#endif /* RCG_H_ */
