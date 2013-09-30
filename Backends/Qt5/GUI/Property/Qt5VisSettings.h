#ifndef QT5VISSETTINGS_H_
#define QT5VISSETTINGS_H_

#include "Qt5Container.h"

namespace GUI {
namespace Property {

class Qt5VisSettings : public Qt5Container {
	public:
		typedef std::shared_ptr<Qt5VisSettings> Ptr;
		typedef std::weak_ptr<Qt5VisSettings>   WPtr;

	public:
		Qt5VisSettings();
		virtual ~Qt5VisSettings();

		//QWidget*  widget();

	protected:
		QWidget*  m_page;
};

} // Property
} // GUI

#endif /* QT5VISSETTINGS_H_ */
