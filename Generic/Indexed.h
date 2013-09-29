#ifndef COMMONINDEXED_H_
#define COMMONINDEXED_H_

namespace Generic {

template <class Type>
class Indexed : public std::pair<int, Type> {
	protected:
		typedef std::pair<int, Type> Base;
	public:
		using Base::pair;

		const int& index() const { return this->first; }
		const Type& value() const { return this->second; }
		int& index() { return this->first; }
		Type& value() { return this->second; }
};


} // Generic

#endif /* COMMONINDEXED_H_ */
