#ifndef QT5HANDLE_H_
#define QT5HANDLE_H_

#include <GUI/Mode/Handle.h>
#include <QtWidgets/QToolBar>

namespace GUI {
namespace Mode {

class Qt5Handle : public Handle {
	public:
		typedef std::shared_ptr<Qt5Handle> Ptr;
		typedef std::weak_ptr<Qt5Handle>   WPtr;

	public:
		Qt5Handle(QToolBar* toolbar, Log::Ptr log);
		virtual ~Qt5Handle();

	protected:
		Group::Ptr createGroup();

	protected:
		QToolBar* m_toolbar;
};


} // Mode
} // GUI

#endif /* QT5HANDLE_H_ */
