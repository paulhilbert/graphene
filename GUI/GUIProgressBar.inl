inline ProgressBar::ProgressBar(std::string label, int steps) : m_label(label), m_steps(steps) {
}

inline ProgressBar::~ProgressBar() {
}

inline void ProgressBar::setSteps(int steps) {
	if (steps > 0) m_steps = steps;
}

inline void ProgressBar::poll(unsigned int done, unsigned int todo) {
	if (todo < 1) return;
	if (done != 0 && done != todo && done % m_steps) return;
	float progress = static_cast<float>(done) / todo;
	poll(progress > 1.f ? 1.f : progress);
}
