/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5GROUP_H_
#define QT5GROUP_H_

#include <GUI/Mode/ModeGroup.h>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QAction>

namespace GUI {
namespace Mode {

class Qt5Group : public Group {
	public:
		typedef std::shared_ptr<Qt5Group> Ptr;
		typedef std::weak_ptr<Qt5Group> WPtr;
		using Group::Callback;

	public:
		Qt5Group(QToolBar* toolbar, Log::Ptr log);
		virtual ~Qt5Group();

	protected:
		Option::Ptr createOption(std::string id, std::string label, fs::path icon);

	protected:
		QToolBar*  m_toolbar;
		QAction*   m_separator;

};

} // Mode
} // GUI

#endif /* QT5GROUP_H_ */
