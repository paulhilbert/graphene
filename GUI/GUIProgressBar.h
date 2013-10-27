#ifndef ABSTRACTPROGRESSBAR_H_
#define ABSTRACTPROGRESSBAR_H_

#include <iostream>
#include <memory>
#include <string>

namespace GUI {

class ProgressBar {
	public:
		typedef std::shared_ptr<ProgressBar> Ptr;
		typedef std::weak_ptr<ProgressBar>   WPtr;

	public:
		ProgressBar(std::string label, int steps = 1);
		virtual ~ProgressBar();

		void setSteps(int steps);

		virtual void poll(float progress) = 0;
		void poll(unsigned int done, unsigned int todo);

	protected:
		std::string  m_label;
		int          m_steps;
};

#include "GUIProgressBar.inl"

} // GUI

#endif /* ABSTRACTPROGRESSBAR_H_ */
