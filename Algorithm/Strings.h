#ifndef ALGORITHMSTRINGS_H_
#define ALGORITHMSTRINGS_H_

#include <string>
#include <vector>
#include <boost/regex.hpp>

namespace Algorithm {

std::string join(const std::vector<std::string>& strings, const std::string& delim = "");
std::vector<std::string> split(const std::string& str, const std::string& regex);

#include "Strings.inl"

} // Algorithm

#endif /* ALGORITHMSTRINGS_H_ */
