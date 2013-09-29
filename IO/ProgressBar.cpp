#include "ProgressBar.h"

#include <cstdio>

namespace IO {

ProgressBar::ProgressBar(int steps, int width, bool printPercent) : m_steps(steps), m_width(width), m_printPercent(printPercent) {
}

std::string ProgressBar::poll(float progress) {
	if (progress >= 1.f) return std::string("Done.");
	char sB[m_width+1]; sB[0] = sB[m_width-1] = '|'; sB[m_width] = '\0';
	for (int i=1; i< m_width-1; ++i) 
		sB[i] = (i<=static_cast<int>(progress*(m_width-2))?'=':' ');
	if (!m_printPercent) return std::string(sB);
	char s[m_width+6];
	int pI = static_cast<int>(progress * 100.f);
	sprintf(s, "%s %3d%%", sB, pI);
	return std::string(s);
}

std::string ProgressBar::poll(unsigned int done, unsigned int todo) {
	if (!todo) return "[Wrong parameter to poll function - todo == 0]";
	float progress = static_cast<float>(done) / todo;
	return poll(progress > 1.f ? 1.f : progress);
}

void ProgressBar::print(unsigned int done, unsigned int todo, std::string prefix, std::string suffix, const PrintFunc& printFunc) {
	if (done != 0 && done != todo && done % m_steps) return;
	printFunc(prefix + (prefix == "" ? "" : "   ") + poll(done, todo)+suffix, done == todo ? Flags::LASTUPDATE : Flags::UPDATE);
}

void ProgressBar::printVerbose(unsigned int done, unsigned int todo, std::string prefix, std::string suffix) {
	print(done, todo, prefix, suffix, std::bind(&Log::verbose, std::placeholders::_1, std::placeholders::_2, true));
}

} // IO
