/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5SEPARATOR_H_
#define QT5SEPARATOR_H_

#include <QtWidgets/QFrame>
#include <GUI/Property/Separator.h>

namespace GUI {
namespace Property {

class Qt5Separator : public Separator {
	public:
		typedef std::shared_ptr<Qt5Separator> Ptr;
		typedef std::weak_ptr<Qt5Separator> WPtr;

	public:
		Qt5Separator();
		virtual ~Qt5Separator();

		void show();
		void hide();
		bool visible() const;
		void enable();
		void disable();
		bool enabled() const;

		QWidget* widget();

	protected:
		QFrame* m_frame;
};


} // Property
} // GUI

#endif /* QT5SEPARATOR_H_ */
