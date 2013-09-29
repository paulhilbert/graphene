#ifndef RECURSE_H_
#define RECURSE_H_

#include <algorithm>
#include <functional>
#include <mutex>
#include <future>

namespace MT {

class Recurse {
	public:
		Recurse(std::launch policy = std::launch::async) : m_policy(policy) {}
		virtual ~Recurse() {}
		
		template <class CIterIn, class IterOut, class Computation>
		void parallel(CIterIn begIn, CIterIn endIn, IterOut begOut, unsigned int thres, const Computation& comp) const;

		template <class CIterIn, class Container, class Computation>
		void push(CIterIn begIn, CIterIn endIn, Container& out, unsigned int thres, const Computation& comp) const;

		template <class CIterIn, class Container, class Computation>
		void append(CIterIn begIn, CIterIn endIn, Container& out, unsigned int thres, const Computation& comp) const;

	protected:
		mutable std::mutex m_mutex;
		std::launch m_policy;
};

#include "Recurse.inl"


} // MT

#endif /* RECURSE_H_ */
