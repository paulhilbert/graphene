#ifndef QT5ADDVISDIALOG_H_
#define QT5ADDVISDIALOG_H_

#include <memory>
#include <string>
#include <vector>

#include <QtWidgets/QDialog>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QVBoxLayout>

#include "Property/Qt5FactorySettings.h"
using GUI::Property::Qt5FactorySettings;

namespace GUI {

class Qt5AddVisDialog : public QDialog {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5AddVisDialog> Ptr;
		typedef std::weak_ptr<Qt5AddVisDialog> WPtr;

	public:
		Qt5AddVisDialog(std::string title, bool singleMode);
		virtual ~Qt5AddVisDialog();

		Qt5FactorySettings::Ptr addFactory(std::string name);

		std::string getActiveFactory() const;
		std::string getActiveVisName() const;
	
	public slots:
		void changeVis(QListWidgetItem* current, QListWidgetItem* previous);

	protected:
		bool                       m_singleMode;
		QListWidget*               m_visSelection;
		QStackedWidget*            m_settingPages;
		std::vector<std::string>   m_factoryNames;
		std::vector<Qt5FactorySettings::Ptr> m_settings;

};


} // GUI

#endif /* QT5ADDVISDIALOG_H_ */
