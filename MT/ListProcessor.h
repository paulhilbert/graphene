#ifndef LISTPROCESSOR_H
#define LISTPROCESSOR_H

#include <functional>
#include <memory>

#include <boost/thread.hpp>
#include <boost/date_time.hpp>


/* THIS IS A QUICK BUT WORKING IMPLEMENTATION - HOWEVER USING THE threadpool LIBRARY SHOULD BE MORE CONVENIENT */

namespace MT {

class ListProcessor {
	public:
		ListProcessor(int numCores);

		template <class ListType>
		void process(typename ListType::iterator begin, typename ListType::iterator end, std::function<void (typename ListType::value_type&)> func);

	protected:
		typedef std::shared_ptr<boost::thread>  Thread;
		typedef std::vector<Thread>             Slots;
	protected:
		Slots m_slots;
};

ListProcessor::ListProcessor(int numCores) {
	m_slots = Slots(numCores);
}

template <class ListType>
void ListProcessor::process(typename ListType::iterator begin, typename ListType::iterator end, std::function<void (typename ListType::value_type&)> func) {
	unsigned int numObjects = std::distance(begin, end);
	unsigned int slotsUsed = numObjects < m_slots.size() ? numObjects : m_slots.size();
	typename ListType::iterator it = begin;
	while (numObjects) {
		for (unsigned int s=0; s<slotsUsed; ++s) {
			if (!m_slots[s] || m_slots[s]->timed_join(boost::posix_time::milliseconds(10))) {
				if (!numObjects) break;
				Thread t(new boost::thread(std::bind(func, *(it++))));
				m_slots[s] = t;
				--numObjects;
			}
		}
	}

	// we fed all objects into slots; wait for remaining slots to finish
	for (unsigned int s = 0; s < m_slots.size(); ++s) {
		if (m_slots[s]) m_slots[s]->join();
	}
}

} // MT

#endif // LISTPROCESSOR_H
