#ifndef SLURP_H
#define SLURP_H

#include <iostream>
#include <fstream>

#include "../IO/Log.h"
using IO::Log;

namespace StringUtils {

struct Slurp {
	static std::vector<std::string> getLines(std::string filename);
	static std::string getContent(std::string filename);
};


std::vector<std::string> Slurp::getLines(std::string filename) {
	std::vector<std::string> lines;
	std::ifstream in(filename.c_str());
	if (!in.good()) {
		Log::error("Could not open input file stream");
		return lines;
	}
	std::string line;
	while (std::getline(in,line)) lines.push_back(line);
	in.close();

	return lines;
}

std::string Slurp::getContent(std::string filename) {
	std::ifstream in(filename.c_str());
	if (!in.good()) {
		Log::error("Could not open input file stream");
		return std::string("");
	}
	std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

	return content;
}
	
} /* StringUtils */

#endif // SLURP_H
