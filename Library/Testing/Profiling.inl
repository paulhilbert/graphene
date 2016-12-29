inline Profiling::Profiling() {
}

#ifdef PROFILING
inline void Profiling::start(std::string name) {
	if (m_profiles.find(name) != m_profiles.end()) {
		m_profiles[name].start = Clock::now();
		m_profiles[name].finished = false;
	} else {
		TimeInfo t;
		t.start = Clock::now();
		t.passed = Duration(0);
		t.finished = false;
		m_profiles[name] = t;
	}
}

inline void Profiling::end(std::string name) {
	if (!m_profiles.count(name)) return;

	m_profiles[name].passed += Clock::now() - m_profiles[name].start;
	m_profiles[name].finished = true;
}

inline void Profiling::reset(std::string name) {
	if (m_profiles.find(name) != m_profiles.end()) {
		m_profiles[name].passed = Duration(0);
	}
}

inline void Profiling::profile(std::string name) {
	if (m_profiles.count(name) && !m_profiles[name].finished) {
		end(name);
	} else {
		start(name);
	}
}

template <class DurationType>
inline void Profiling::summarize(std::function<void (std::string)> logFunction) {
	if (!m_profiles.size()) return;
	std::function<void (std::string)> log = logFunction ? std::move(logFunction) : [&] (std::string msg) { std::cout << msg << "\n"; };

	for (auto it = m_profiles.begin(); it != m_profiles.end(); ++it) {
		if (!(*it).second.finished) {
			log("Unfinished profile block: " + (*it).first);
			end((*it).first);
		}
	}

	// print empty lines before summary
	log("");
	log("Profiling summary");

	typedef typename DurationType::rep Rep;
	Rep overall(0);
	for (auto it = m_profiles.begin(); it != m_profiles.end(); ++it) {
		Rep duration = std::chrono::duration_cast<DurationType>(it->second.passed).count();
		overall += duration;
		log(format<DurationType>(it->first, duration));
		//double fraction = (double)(duration.total_microseconds()) / profileSpan.get().length().total_microseconds();
	}
	log(format<DurationType>("Summed: ", overall));
	log("Finished summary");
	m_profiles.clear();
}

template <class DurationType>
typename DurationType::rep Profiling::duration(std::string profile) const {
	auto findIt = m_profiles.find(profile);
	if (findIt == m_profiles.end()) {
		return typename DurationType::rep(0);
	}
	return std::chrono::duration_cast<DurationType>(findIt->second.passed).count();
}

#else

inline void Profiling::start(std::string) {
}

inline void Profiling::end(std::string) {
}

inline void Profiling::reset(std::string) {
}

inline void Profiling::profile(std::string) {
}

template <class DurationType>
inline void Profiling::summarize(std::function<void (std::string)>) {
}

template <class DurationType>
typename DurationType::rep Profiling::duration(std::string profile) const {
	return typename DurationType::rep(0);
}

#endif // PROFILING


template <class DurationType>
inline std::string Profiling::format(std::string name, typename DurationType::rep duration) {
	char output[255];
	std::string format = "  %-25s%11d"+unit<DurationType>();//   %13.3f%%";
	sprintf(output, format.c_str(), name.c_str(), duration); //, fraction*100.0);
	std::string result(output);
	return result;
}

template <>
inline std::string Profiling::unit<std::chrono::hours>() {
	return "h";
}

template <>
inline std::string Profiling::unit<std::chrono::minutes>() {
	return "m";
}

template <>
inline std::string Profiling::unit<std::chrono::seconds>() {
	return "s";
}

template <>
inline std::string Profiling::unit<std::chrono::milliseconds>() {
	return "ms";
}

template <>
inline std::string Profiling::unit<std::chrono::microseconds>() {
	return "us";
}

template <>
inline std::string Profiling::unit<std::chrono::nanoseconds>() {
	return "ns";
}
