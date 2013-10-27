#ifndef ABSTRACTPROGRESSBARPOOL_H_
#define ABSTRACTPROGRESSBARPOOL_H_

#include <mutex>

#include "GUIProgressBar.h"

namespace GUI {

class ProgressBarPool {
	public:
		typedef std::shared_ptr<ProgressBarPool> Ptr;
		typedef std::weak_ptr<ProgressBarPool>   WPtr;

	public:
		ProgressBarPool();
		virtual ~ProgressBarPool();

		ProgressBar::Ptr create(std::string label, int steps = 1);
		void remove(int index);

		void  setBarCountChangeCallback(std::function<void (int)> callback);

	protected:
		virtual ProgressBar::Ptr createProgressBar(std::string label, int steps) = 0;
		virtual void removeProgressBar(int index) = 0;

	protected:
		unsigned int               m_count;
		std::mutex                 m_mutex;
		std::function<void (int)>  m_callback;
};

#include "GUIProgressBarPool.inl"

} // GUI

#endif /* ABSTRACTPROGRESSBARPOOL_H_ */
