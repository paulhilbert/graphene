#ifndef QT5PROGRESSBARHANDLE_H_
#define QT5PROGRESSBARHANDLE_H_

#include <QtCore/QObject>

#include <IO/AbstractProgressBar.h>
#include <IO/AbstractProgressBarPool.h>

namespace GUI {

class Qt5ProgressBarHandle : public QObject, public IO::AbstractProgressBar {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5ProgressBarHandle> Ptr;
		typedef std::weak_ptr<Qt5ProgressBarHandle> WPtr;

	public:
		Qt5ProgressBarHandle(IO::AbstractProgressBarPool* pool, int idx, std::string label, int steps = 1);
		virtual ~Qt5ProgressBarHandle();

		void poll(float progress);

	signals:
		void polled(float progress);

	protected:
		IO::AbstractProgressBarPool* m_pool;
		int m_idx;
};


} // GUI

#endif /* QT5PROGRESSBARHANDLE_H_ */
