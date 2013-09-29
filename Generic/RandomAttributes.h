#ifndef RANDOMATTRIBUTES_H_
#define RANDOMATTRIBUTES_H_

#include <iostream>
using std::string;

#include <map>
using std::map;

using std::pair;

#include <sstream>
using std::stringstream;

#include <boost/optional.hpp>
#include <boost/none.hpp>
#include <boost/any.hpp>
using boost::optional;
using boost::none;
using boost::any;
using boost::any_cast;

namespace Generic {

class RandomAttributes {
	public:
		RandomAttributes( ) {}
		virtual ~RandomAttributes() {}

		inline any& operator[](string name) {
			return m_attributes[name];
		}

		template <class T>
		boost::optional<T> get(string name) {
			optional<T> result = none;
			if (m_attributes.find(name) == m_attributes.end()) return result;
			try {
				result = any_cast<T>(m_attributes[name]);
			} catch (...) {	}

			return result;
		}

		inline map<string,any>& getAttributes() { return m_attributes; }

		inline void addTypedAttribute(string name, string type, string value) {
			stringstream str; str << value;
			if      (type == "int")   { int v; str >> v; m_attributes[name] = static_cast<int>(v); return; }
			else if (type == "float") { float v; str >> v; m_attributes[name] = static_cast<float>(v); return; }

			m_attributes[name] = str.str();
		}
		inline void removeAttribute(string name) {
			// map::erase will silently fail if key doesn't exist.
			m_attributes.erase(name);
		}
		inline optional< pair<string,string> > getTypedAttributeStr(string name) {
			stringstream v;
			// try int
			optional<int> i = get<int>(name);
			if (i) { v << i.get(); return make_pair(v.str(), string("int")); }
			// try float
			optional<float> f = get<float>(name);
			if (f) { v << f.get(); return make_pair(v.str(), string("float")); }
			// try bool
			optional<bool> b = get<bool>(name);
			if (b) { v << (b.get() ? "true" : "false"); return make_pair(v.str(), string("bool")); }
			// try string
			optional<string> s = get<string>(name);
			if (s) { return make_pair(s.get(), string("string")); }
			// we failed
			return none;
		}

	protected:
		map<string,any> m_attributes;
};

}

#endif /* RANDOMATTRIBUTES_H_ */
