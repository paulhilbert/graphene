/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5SETTINGS_H_
#define QT5SETTINGS_H_

#include <include/common.h>
#include <QtWidgets/QTabWidget>

#include "Property/Qt5VisSettings.h"
using GUI::Property::Qt5VisSettings;
using GUI::Property::Container;

#include <GUI/GUILog.h>


namespace GUI {

class Qt5Settings {
	public:
		typedef std::shared_ptr<Qt5Settings> Ptr;
		typedef std::weak_ptr<Qt5Settings> WPtr;

	public:
		Qt5Settings(Log::Ptr log);
		virtual ~Qt5Settings();

		Container::Ptr add(std::string name, bool hasActiveCheckBox = true);
		Container::Ptr get(std::string name);
		std::vector<std::string> getActiveTabs() const;
		std::string remove(int index);

		QWidget* widget();

	protected:
		Log::Ptr                          m_log;
		std::vector<Qt5VisSettings::Ptr>  m_tabs;
		QTabWidget*                       m_tabWidget;
		std::vector<std::string>          m_indexMap;
};

} // GUI

#endif /* QT5SETTINGS_H_ */
