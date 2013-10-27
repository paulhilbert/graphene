inline ProgressBarPool::ProgressBarPool() : m_count(0) {
}

inline ProgressBarPool::~ProgressBarPool() {
}

inline ProgressBar::Ptr ProgressBarPool::create(std::string label, int steps) {
	std::lock_guard<std::mutex> guard(m_mutex);
	if (m_callback) m_callback(++m_count);
	auto bar = createProgressBar(label, steps);
	return bar;
}

inline void ProgressBarPool::remove(int index) {
	std::lock_guard<std::mutex> guard(m_mutex);
	removeProgressBar(index);
	if (m_callback) m_callback(--m_count);
}

inline void ProgressBarPool::setBarCountChangeCallback(std::function<void (int)> callback) {
	m_callback = std::move(callback);
}
