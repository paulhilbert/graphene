#include "Qt5FactorySettings.h"

namespace GUI {
namespace Property {

Qt5FactorySettings::Qt5FactorySettings(std::string name) : Qt5Container(), m_group(new QGroupBox(QString::fromStdString(name + " settings"))) {
	setWidget(m_group);
}

Qt5FactorySettings::~Qt5FactorySettings() {
}

//QWidget* Qt5FactorySettings::widget() {
//	return m_area;
//}

} // Property
} // GUI
