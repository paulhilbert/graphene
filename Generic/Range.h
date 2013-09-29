#ifndef GENERICRANGE_H_
#define GENERICRANGE_H_

#include <utility>

namespace Generic {

template <class Iter>
class Range : public std::pair<Iter, Iter> {
	public:
		Range(std::pair<Iter,Iter> const& x) : std::pair<Iter,Iter>(x) {}
		virtual ~Range() {}

		Iter begin() const { return this->first;  }
		Iter end()   const { return this->second; }
};


} // Generic

#endif /* GENERICRANGE_H_ */
