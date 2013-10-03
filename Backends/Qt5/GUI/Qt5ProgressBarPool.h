#ifndef QT5PROGRESSBARPOOL_H_
#define QT5PROGRESSBARPOOL_H_

#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>

#include <IO/AbstractProgressBarPool.h>
#include "Qt5ProgressBar.h"

namespace GUI {

class Qt5ProgressBarPool : public QObject, public IO::AbstractProgressBarPool {
	Q_OBJECT

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

	protected:
		IO::AbstractProgressBar::Ptr createProgressBar(std::string label, int steps);
		void removeProgressBar(int index);

	protected:
		QWidget*                          m_area;
		QVBoxLayout*                      m_box;
		std::vector<Qt5ProgressBar::Ptr>  m_bars;
};

} // GUI

#endif /* QT5PROGRESSBARPOOL_H_ */
