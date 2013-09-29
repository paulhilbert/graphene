template <class CIterIn, class IterOut, class Computation>
void Recurse::parallel(CIterIn begIn, CIterIn endIn, IterOut begOut, unsigned int thres, const Computation& comp) const {
	unsigned int len = std::distance(begIn, endIn);
	if (len < thres) {
		for (auto it = begIn; it != endIn; ++it) *(begOut++) = comp(it);
		return;
	}
	CIterIn  midIn  = begIn;
	IterOut midOut = begOut;
	std::advance(midIn,  len/2);
	std::advance(midOut, len/2);
	auto  left = std::async(m_policy, std::bind(&Recurse::parallel<CIterIn,IterOut,Computation>, this, begIn, midIn, begOut, thres, std::cref(comp)));
	auto right = std::async(m_policy, std::bind(&Recurse::parallel<CIterIn,IterOut,Computation>, this, midIn, endIn, midOut, thres, std::cref(comp)));
	left.get();
	right.get();
}

template <class CIterIn, class Container, class Computation>
void Recurse::push(CIterIn begIn, CIterIn endIn, Container& out, unsigned int thres, const Computation& comp) const {
	unsigned int len = std::distance(begIn, endIn);
	if (len < thres) {
		for (auto itIn = begIn; itIn != endIn; ++itIn) {
			auto result = comp(itIn);
			{
				std::lock_guard<std::mutex> lg(m_mutex);
				out.push_back(result);
			}
		}
		return;
	}
	CIterIn midIn = begIn;
	std::advance(midIn,  len/2);
	auto  left = std::async(m_policy, std::bind(&Recurse::push<CIterIn,Container,Computation>, this, begIn, midIn, std::ref(out), thres, std::cref(comp)));
	auto right = std::async(m_policy, std::bind(&Recurse::push<CIterIn,Container,Computation>, this, midIn, endIn, std::ref(out), thres, std::cref(comp)));
	left.get();
	right.get();
}

template <class CIterIn, class Container, class Computation>
void Recurse::append(CIterIn begIn, CIterIn endIn, Container& out, unsigned int thres, const Computation& comp) const {
	unsigned int len = std::distance(begIn, endIn);
	if (len < thres) {
		for (auto itIn = begIn; itIn != endIn; ++itIn) {
			auto result = comp(itIn);
			{
				std::lock_guard<std::mutex> lg(m_mutex);
				out.insert(out.end(), result.begin(), result.end());
			}
		}
		return;
	}
	CIterIn midIn = begIn;
	std::advance(midIn,  len/2);
	auto  left = std::async(m_policy, std::bind(&Recurse::append<CIterIn,Container,Computation>, this, begIn, midIn, std::ref(out), thres, std::cref(comp)));
	auto right = std::async(m_policy, std::bind(&Recurse::append<CIterIn,Container,Computation>, this, midIn, endIn, std::ref(out), thres, std::cref(comp)));
	left.get();
	right.get();
}

/*
template <class CIterIn, class IterOut>
void Recurse::parallel(CIterIn begIn, CIterIn endIn, IterOut begOut, const Comp<typename IterOut::value_type, CIterIn>& comp, unsigned int thres) const {
	unsigned int len = std::distance(begIn, endIn);
	if (len < thres) {
		for (auto it = begIn; it != endIn; ++it) *(begOut++) = comp(it);
		return;
	}
	CIterIn  midIn  = begIn;
	IterOut midOut = begOut;
	std::advance(midIn,  len/2);
	std::advance(midOut, len/2);
	auto  left = std::async(m_policy, std::bind(&Recurse::parallel<CIterIn,IterOut>, this, begIn, midIn, begOut, std::cref(comp), thres));
	auto right = std::async(m_policy, std::bind(&Recurse::parallel<CIterIn,IterOut>, this, midIn, endIn, midOut, std::cref(comp), thres));
	left.get();
	right.get();
}

template <class CIterIn, class Container>
void Recurse::push(CIterIn begIn, CIterIn endIn, Container& out, const Comp<typename Container::value_type,CIterIn>& comp, unsigned int thres) const {
	unsigned int len = std::distance(begIn, endIn);
	if (len < thres) {
		for (auto itIn = begIn; itIn != endIn; ++itIn) {
			auto result = comp(itIn);
			{
				std::lock_guard<std::mutex> lg(m_mutex);
				out.push_back(result);
			}
		}
		return;
	}
	CIterIn midIn = begIn;
	std::advance(midIn,  len/2);
	auto  left = std::async(m_policy, std::bind(&Recurse::push<CIterIn,Container>, this, begIn, midIn, std::ref(out), std::cref(comp), thres));
	auto right = std::async(m_policy, std::bind(&Recurse::push<CIterIn,Container>, this, midIn, endIn, std::ref(out), std::cref(comp), thres));
	left.get();
	right.get();
}

template <class CIterIn, class Container>
void Recurse::append(CIterIn begIn, CIterIn endIn, Container& out, const Comp<Container,CIterIn>& comp, unsigned int thres) const {
	unsigned int len = std::distance(begIn, endIn);
	if (len < thres) {
		for (auto itIn = begIn; itIn != endIn; ++itIn) {
			auto result = comp(itIn);
			{
				std::lock_guard<std::mutex> lg(m_mutex);
				out.insert(out.end(), result.begin(), result.end());
			}
		}
		return;
	}
	CIterIn midIn = begIn;
	std::advance(midIn,  len/2);
	auto  left = std::async(m_policy, std::bind(&Recurse::append<CIterIn,Container>, this, begIn, midIn, std::ref(out), std::cref(comp), thres));
	auto right = std::async(m_policy, std::bind(&Recurse::append<CIterIn,Container>, this, midIn, endIn, std::ref(out), std::cref(comp), thres));
	left.get();
	right.get();
}
*/
