#ifndef ABSTRACTPROGRESSBARPOOL_H_
#define ABSTRACTPROGRESSBARPOOL_H_

#include "GUIProgressBar.h"

namespace GUI {

class ProgressBarPool {
	public:
		typedef std::shared_ptr<ProgressBarPool> Ptr;
		typedef std::weak_ptr<ProgressBarPool>   WPtr;

	public:
		ProgressBarPool() {}
		virtual ~ProgressBarPool() {}

		virtual ProgressBar::Ptr create(std::string label, int steps = 1) = 0;
};

} // GUI

#endif /* ABSTRACTPROGRESSBARPOOL_H_ */
