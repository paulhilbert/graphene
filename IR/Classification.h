#ifndef CLASSIFICATION_H
#define CLASSIFICATION_H

#include <iostream>
#include <vector>
#include <set>
using std::pair;
using std::vector;

#include <algorithm>
using std::transform;
using std::for_each;

#include <functional>

#include <fstream>
using std::string;
using std::ifstream;
using std::ofstream;

#include <tuple>
using std::pair;
using std::tie;

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <boost/optional.hpp>
#include <boost/none.hpp>
using boost::optional;
using boost::none;

#include <Random/RNG.h>
using Random::RNG;

#include <IO/Log.h>
using IO::Log;

#include "Class.h"


namespace IR {

template <class TClassId, class TObjId, class TObject>
class Classification {
	public:
		typedef map<TClassId, Class<TClassId,TObjId,TObject>>  ClassMap;
		typedef vector<Class<TClassId,TObjId,TObject>>         ClassVec;

		Classification() {}

	public:
		Classification(std::string training, std::string testing, optional<string> classRegExp = none, optional<string> objRegExp = none, optional<std::set<TObjId>> ignoreList = none);

		ClassVec getClassVector(optional<ClassType> restriction = none, int minClassSize = 1);
		ClassMap getClassMap(optional<ClassType> restriction = none, int minClassSize = 1);
		Class<TClassId,TObjId,TObject>& operator[](TClassId id);
		const Class<TClassId,TObjId,TObject>& operator[](TClassId id) const;
		std::shared_ptr<TObject> getObject(TObjId id);
		optional<ClassType> getObjectType(TObjId id);

		void forEachClass(std::function<void (Class<TClassId,TObjId,TObject>&)> func);

		void randomizeSeparation(float testingFraction = 0.5f);

		void serialize(std::string training, std::string testing);
		pair<std::string,std::string> serialize();  // (training,testing) = serialize

	protected:
		void parseSingleFile(std::string& content, ClassType t, optional<string> classRegExp, optional<string> objRegExp, optional<std::set<TObjId>> ignoreList = none);

	protected:
		ClassMap m_classes;
};


template <class TClassId, class TObjId, class TObject>
Classification<TClassId,TObjId,TObject>::Classification(std::string training, std::string testing, optional<string> classRegExp, optional<string> objRegExp, optional<std::set<TObjId>> ignoreList) {
	parseSingleFile(training, TRAINING, classRegExp, objRegExp, ignoreList);
	parseSingleFile(testing, TESTING, classRegExp, objRegExp, ignoreList);
}

template <class TClassId, class TObjId, class TObject>
typename Classification<TClassId,TObjId,TObject>::ClassVec Classification<TClassId,TObjId,TObject>::getClassVector(optional<ClassType> restriction, int minClassSize) {
	typedef typename Class<TClassId,TObjId,TObject>::Object Obj;
	ClassVec result;
	int dropped = 0;
	std::for_each(m_classes.begin(), m_classes.end(), [&] (typename ClassMap::value_type& c) {
		Class<TClassId,TObjId,TObject> newC = c.second.copy(restriction);
		if (static_cast<int>(newC.getNumObjects()) >= minClassSize)	result.push_back(newC);
		else ++dropped;
	});
	if (dropped) Log::info("Dropped "+lexical_cast<std::string>(dropped)+" classes of insufficient size");
	return result;
}

template <class TClassId, class TObjId, class TObject>
typename Classification<TClassId,TObjId,TObject>::ClassMap Classification<TClassId,TObjId,TObject>::getClassMap(optional<ClassType> restriction, int minClassSize) {
	typedef typename Class<TClassId,TObjId,TObject>::Object Obj;
	ClassMap result;
	int dropped = 0;
	std::for_each(m_classes.begin(), m_classes.end(), [&] (typename ClassMap::value_type& c) {
		Class<TClassId,TObjId,TObject> newC = c.second.copy(restriction);
		if (static_cast<int>(newC.getNumObjects()) >= minClassSize)	result[c.first] = newC;
		else ++dropped;
	});
	if (dropped) Log::info("Dropped "+lexical_cast<std::string>(dropped)+" classes of insufficient size");
	return result;
}

template <class TClassId, class TObjId, class TObject>
Class<TClassId,TObjId,TObject>& Classification<TClassId,TObjId,TObject>::operator[](TClassId id) {
	return m_classes[id];
}

template <class TClassId, class TObjId, class TObject>
const Class<TClassId,TObjId,TObject>& Classification<TClassId,TObjId,TObject>::operator[](TClassId id) const {
	return m_classes[id];
}

template <class TClassId, class TObjId, class TObject>
std::shared_ptr<TObject> Classification<TClassId,TObjId,TObject>::getObject(TObjId id) {
	for (auto it = m_classes.begin(); it != m_classes.end(); ++it) {
		if (it->second.contains(id)) {
			std::shared_ptr<TObject> obj = it->second[id].obj;
			if (obj) return obj;
		}
	}
	return NULL;
}

template <class TClassId, class TObjId, class TObject>
optional<ClassType> Classification<TClassId,TObjId,TObject>::getObjectType(TObjId id) {
	for (auto it = m_classes.begin(); it != m_classes.end(); ++it) {
		if (it->second.contains(id)) {
			return it->second[id].type;
		}
	}
	return none;
}

template <class TClassId, class TObjId, class TObject>
void Classification<TClassId,TObjId,TObject>::forEachClass(std::function<void (Class<TClassId,TObjId,TObject>&)> func) {
	std::for_each(m_classes.begin(), m_classes.end(), [&] (pair<TClassId, Class<TClassId,TObjId,TObject>>& c) { func(c.second); });
}

template <class TClassId, class TObjId, class TObject>
void Classification<TClassId,TObjId,TObject>::randomizeSeparation(float testingFraction) {
	typedef typename Class<TClassId,TObjId,TObject>::Object Obj;
	RNG* rng = RNG::instance();

	for (typename ClassMap::iterator cIt = m_classes.begin(); cIt != m_classes.end(); ++cIt) {
		// first initialize all models to TRAINING
		cIt->second.forEachObj([=] (Obj& o) { o.type = TRAINING; });

		list<TObjId> indices = cIt->second.getIdList();
		unsigned int k = indices.size() * testingFraction;
		typename list<TObjId>::iterator listIt;
		while(k) {
			unsigned int chosen = rng->uniformAB<unsigned int>(0, indices.size());
			listIt = indices.begin();
			advance(listIt, chosen);
			cIt->second[*listIt].type = TESTING;
			indices.erase(listIt);
			--k;
		}
	}
}

template <class TClassId, class TObjId, class TObject>
void Classification<TClassId,TObjId,TObject>::parseSingleFile(std::string& content, ClassType t, optional<string> classRegExp, optional<string> objRegExp, optional<std::set<TObjId>> ignoreList) {
	typedef typename ClassMap::iterator CIter;

	boost::regex exprClass("("+(classRegExp ? classRegExp.get() : ".+?")+"):(.+?)$");
	boost::regex exprIds("("+(objRegExp ? objRegExp.get() : ".+?")+"),?");

	string line;
	vector<string> lines;
	boost::split(lines, content, boost::is_any_of("\n\r"));
	boost::smatch what, whatId;
	for (unsigned int i=0; i<lines.size(); ++i) {
		line = lines[i];
		if (boost::regex_match(line, what, exprClass)) {
			TClassId classId = lexical_cast<TClassId>(what[1]);
			// create class if necessary
			if (m_classes.find(classId) == m_classes.end()) {
				m_classes.insert(make_pair(classId, Class<TClassId,TObjId,TObject>(classId)));
			}

			// parse obj ids
			string idStrings = what[2];
			string::const_iterator start = idStrings.begin();
			string::const_iterator end = idStrings.end();
			while (boost::regex_search(start, end, whatId, exprIds)) {
				TObjId objId = lexical_cast<TObjId>(whatId[1]);
				if (!ignoreList || ignoreList.get().find(objId) == ignoreList.get().end()) {
					m_classes.find(classId)->second.addObject(objId, t);
				}
				start = whatId[0].second;
			}
		}
	}
}

template <class TClassId, class TObjId, class TObject>
void Classification<TClassId,TObjId,TObject>::serialize(std::string training, std::string testing) {
	std::string trainContent, testContent;
	tie(trainContent, testContent) = serialize();

	ofstream out(training);
	if (!out.good()) {
		Log::error("Could not open training output file for writing");
		return;
	}

	out << trainContent;
	out.close();

	out.open(testing);
	if (!out.good()) {
		Log::error("Could not open testing output file for writing");
		return;
	}

	out << testContent;
	out.close();
}

template <class TClassId, class TObjId, class TObject>
pair<std::string,std::string> Classification<TClassId,TObjId,TObject>::serialize() {
	int numObjects;

	std::stringstream out;

	for_each(m_classes.begin(), m_classes.end(), [&](typename ClassMap::value_type& c) {
		// determine total count of relevant objects
		numObjects = 0;
		c.second.forEachObj([&] (typename Class<TClassId,TObjId,TObject>::Object& o) { if (o.type == TRAINING) ++numObjects; });
		if (!numObjects) return;
		// if we have objects, serialize
		out << c.second.getId() << ":";
		c.second.forEachObj([&] (typename Class<TClassId,TObjId,TObject>::Object& o) { if (o.type == TRAINING) out << o.id << ","; });
		out << "\n";
	});
	std::string training = out.str();
	out.str("");

	for_each(m_classes.begin(), m_classes.end(), [&](typename ClassMap::value_type& c) {
		// determine total count of relevant objects
		numObjects = 0;
		c.second.forEachObj([&] (typename Class<TClassId,TObjId,TObject>::Object& o) { if (o.type == TESTING) ++numObjects; });
		if (!numObjects) return;
		// if we have objects, serialize
		out << c.second.getId() << ":";
		c.second.forEachObj([&] (typename Class<TClassId,TObjId,TObject>::Object& o) { if (o.type == TESTING) out << o.id << ","; });
		out << "\n";
	});
	std::string testing = out.str();

	return std::make_pair(training, testing);
}

} // IR

#endif // CLASSIFICATION_H
