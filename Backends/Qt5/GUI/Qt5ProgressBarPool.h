#ifndef QT5PROGRESSBARPOOL_H_
#define QT5PROGRESSBARPOOL_H_

#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>

#include <IO/AbstractProgressBarPool.h>
#include "Qt5ProgressBar.h"

namespace GUI {

class Qt5ProgressBarPool : public IO::AbstractProgressBarPool {
	public:
		typedef std::shared_ptr<Qt5ProgressBarPool> Ptr;
		typedef std::weak_ptr<Qt5ProgressBarPool> WPtr;
		friend class Qt5ProgressBar;

	public:
		Qt5ProgressBarPool();
		virtual ~Qt5ProgressBarPool();

		void      show();
		void      hide();
		QWidget*  widget();
		void      setBarCountChangeCallback(std::function<void (int)> func);

		void      notifyRemove();

	protected:
		IO::AbstractProgressBar::Ptr createProgressBar(std::string label, int steps);
//		void removeProgressBar(int idx);

	protected:
		QWidget*                          m_area;
		QVBoxLayout*                      m_box;
		unsigned int                      m_count;
		std::function<void (int)>         m_onChange;
};

} // GUI

#endif /* QT5PROGRESSBARPOOL_H_ */
