#ifndef STRINGMANIP_H_
#define STRINGMANIP_H_

#include <iostream>
#include <string>
#include <locale>
#include <algorithm>

namespace StringUtils {

class StringManip {
	public:
		typedef std::string::value_type charT;

		static string upper(string str) {
			string result = str;
			std::transform(result.begin(), result.end(), result.begin(), &StringManip::upperChar);
			return result;
		}

	protected:
		static charT upperChar(charT arg) {
			return std::use_facet<std::ctype<charT> >(std::locale()).toupper(arg);
		}
};

} // StringUtils

#endif /* STRINGMANIP_H_ */
