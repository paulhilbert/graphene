/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


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
