#ifndef QT5PROGRESSBARPOOLHANDLE_H_
#define QT5PROGRESSBARPOOLHANDLE_H_

#include <GUI/GUIProgressBarPool.h>

#include <QtCore/QWaitCondition>
#include <QtCore/QMutex>
#include <QtCore/QObject>

namespace GUI {


class Qt5ProgressBarPoolHandle : public QObject, public ProgressBarPool {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5ProgressBarPoolHandle> Ptr;
		typedef std::weak_ptr<Qt5ProgressBarPoolHandle>   WPtr;

	public:
		Qt5ProgressBarPoolHandle();
		virtual ~Qt5ProgressBarPoolHandle();

		ProgressBar::Ptr create(std::string label, int steps = 1);

	signals:
		void sigCreate(QString label, int steps);

	public slots:
		void receiveBar(ProgressBar::Ptr bar);

	protected:
		ProgressBar::Ptr m_savedBar;
		QWaitCondition m_waiter;
};

} // GUI

#endif /* QT5PROGRESSBARPOOLHANDLE_H_ */
