#ifndef IOTYPES_H_
#define IOTYPES_H_

#include <string>

#include <boost/lexical_cast.hpp>
using boost::lexical_cast;

namespace IO {

struct Flags {
	typedef int PrintOptions;
	static PrintOptions CLEAR;
	static PrintOptions FLUSH;
	static PrintOptions END;
	static PrintOptions UPDATE;
	static PrintOptions LASTUPDATE;
};

struct Types {
	typedef enum { RED, GREEN, BLUE, YELLOW, DEFAULT } Color;
	static std::string color(Color col = DEFAULT);
};

} // IO

#endif /* IOTYPES_H_ */
