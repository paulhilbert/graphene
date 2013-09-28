#ifndef QT5SETTINGS_H_
#define QT5SETTINGS_H_

#include <vector>
#include <memory>
#include <QtWidgets/QTabWidget>

#include "Property/Qt5VisSettings.h"
using GUI::Property::Qt5VisSettings;
using GUI::Property::Container;

#include <GUI/Log.h>


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
