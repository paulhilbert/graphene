#ifndef QT5PROGRESSBAR_H_
#define QT5PROGRESSBAR_H_

#include <QtWidgets/QWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QProgressBar>

#include <IO/AbstractProgressBar.h>
#include <IO/AbstractProgressBarPool.h>

namespace GUI {

class Qt5ProgressBar : public IO::AbstractProgressBar {
	public:
		typedef std::shared_ptr<Qt5ProgressBar> Ptr;
		typedef std::weak_ptr<Qt5ProgressBar> WPtr;

	public:
		Qt5ProgressBar(IO::AbstractProgressBarPool* pool, int idx, std::string label, int steps = 1);
		virtual ~Qt5ProgressBar();

		void poll(float progress);

		QWidget* widget();
		int      index() const;

	protected:
		IO::AbstractProgressBarPool* m_pool;
		int           m_idx;
		QWidget*      m_area;
		QHBoxLayout*  m_box;
		QLabel*       m_labelWidget;
		QProgressBar* m_bar;
};


} // GUI

#endif /* QT5PROGRESSBAR_H_ */
