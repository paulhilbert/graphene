
inline std::string join(const std::vector<std::string>& strings, const std::string& delim) {
	if (!strings.size()) return std::string("");
	std::string result = strings[0];
	auto iter = strings.begin(); ++iter;
	for (; iter != strings.end(); ++iter) {
		result += delim;
		result += *iter;
	}
	return result;
}

inline std::vector<std::string> split(const std::string& str, const std::string& regex) {
	boost::sregex_token_iterator first(str.begin(), str.end(), boost::basic_regex<char>(regex), -1), last;
	return std::vector<std::string>(first, last);
}
