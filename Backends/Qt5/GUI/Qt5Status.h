#ifndef QT5STATUS_H_
#define QT5STATUS_H_

#include <GUI/Status.h>
#include <QtWidgets/QMainWindow>

namespace GUI {

class Qt5Status : public Status {
	public:
		typedef std::shared_ptr<Qt5Status> Ptr;
		typedef std::weak_ptr<Qt5Status>   WPtr;

	public:
		Qt5Status(QMainWindow* mainWindow);
		virtual ~Qt5Status();

		void         set(const std::string& text);
		std::string  get() const;
		void         clear();

	protected:
		QMainWindow* m_mainWindow;
};


} // GUI

#endif /* QT5STATUS_H_ */
