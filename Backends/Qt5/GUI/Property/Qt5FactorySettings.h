#ifndef QT5FACTORYSETTINGS_H_
#define QT5FACTORYSETTINGS_H_

#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>
#include "Qt5Container.h"

namespace GUI {
namespace Property {

class Qt5FactorySettings : public Qt5Container {
	public:
		typedef std::shared_ptr<Qt5FactorySettings> Ptr;
		typedef std::weak_ptr<Qt5FactorySettings>   WPtr;

	public:
		Qt5FactorySettings(std::string name);
		virtual ~Qt5FactorySettings();

		//QWidget* widget();

	protected:
		QGroupBox*   m_group;
};

} // Property
} // GUI

#endif /* QT5FACTORYSETTINGS_H_ */
