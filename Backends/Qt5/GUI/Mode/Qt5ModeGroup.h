#ifndef QT5GROUP_H_
#define QT5GROUP_H_

#include <GUI/Mode/ModeGroup.h>
#include <QtWidgets/QToolBar>

namespace GUI {
namespace Mode {

class Qt5Group : public Group {
	public:
		typedef std::shared_ptr<Qt5Group> Ptr;
		typedef std::weak_ptr<Qt5Group> WPtr;
		using Group::Callback;

	public:
		Qt5Group(QToolBar* toolbar, Log::Ptr log, const Callback& onChange = nullptr);
		virtual ~Qt5Group();

	protected:
		Option::Ptr createOption(std::string id, std::string label, fs::path icon);

	protected:
		QToolBar*  m_toolbar;

};

} // Mode
} // GUI

#endif /* QT5GROUP_H_ */
