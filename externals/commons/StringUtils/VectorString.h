#ifndef VECTORSTRING_H_
#define VECTORSTRING_H_

#include <vector>
using std::vector;

#include <boost/lexical_cast.hpp>
using boost::lexical_cast;
#include <boost/algorithm/string.hpp>

#include <Math/Vector.h>
using namespace Math;

namespace StringUtils {

// Creates a string out of given std::vector
template <typename T>
inline string vec2Str(const std::vector<T>& vec, std::string delimiter = std::string(",")) {
	std::vector<string> s(vec.size());
	std::transform(vec.begin(), vec.end(), s.begin(), lexical_cast<string,T>);
	return boost::algorithm::join(s, delimiter);
}

// Creates a string out of given Vector
template <typename T, int n>
inline string Vec2Str(const Vector<T, n>& vec) {
	std::stringstream result;
	for (int i = 0; i < n; ++i) {
		result << vec[i];
		if (i != n - 1)
			result << ",";
	}
	return result.str();
}

// Creates a string out of a vector of Vector
template <typename T, int n>
inline string Vecs2Str(const vector< Vector<T, n> >& vecs) {
	std::stringstream result;
	for (size_t i = 0; i < vecs.size(); ++i) {
		result << Vec2Str<T, n>(vecs[i]);
		if (i != vecs.size() - 1)
			result << ";";
	}
	return result.str();
}

// Creates a std::vector out of given string
template <typename T>
inline std::vector<T> Str2vec(const string str, std::string delimiter = std::string(", ")) {
	// WARNING: this function is not tested!
	std::vector<std::string> tokens;
	boost::split(tokens, str, boost::is_any_of(delimiter));
	std::vector<T> result(tokens.size());
	std::transform(tokens.begin(), tokens.end(), result.begin(), [&](std::string t) { return boost::lexical_cast<T>(t); });
	return result;
}


// Creates a Vector out of given string
template <typename T, int n>
inline Vector<T, n> Str2Vec(const string str) {
	string workstr(str);
	Vector<T, n> result;
	for (int i = 0; i < n; ++i) {
		std::stringstream temp;
		size_t pos = workstr.find(",");
		if (pos == workstr.npos) {
			temp << workstr;
			temp >> result[i];
		} else {
			temp << workstr.substr(0, pos);
			temp >> result[i];
			workstr.replace(0, pos + 1, "");
		}
	}
	return result;
}

// Creates a vector of Vector out of given string
template <typename T, int n>
inline vector< Vector<T, n> > Str2Vecs(const string str) {
	string workstr(str);
	vector< Vector<T, n> > result;

	// Return empty vector if string is completely empty
	if ( workstr.empty() )
		return result;

	size_t pos = workstr.find(";");
	while (pos != workstr.npos) {
		result.push_back( Str2Vec<T, n>(workstr.substr(0, pos)) );
		workstr.replace(0, pos + 1, "");
		pos = workstr.find(";");
	}
	result.push_back( Str2Vec<T, n>(workstr) );
	return result;
}

} // StringUtils

#endif /* VECTORSTRING_H_ */
