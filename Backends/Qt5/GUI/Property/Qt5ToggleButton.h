/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5TOGGLEBUTTON_H_
#define QT5TOGGLEBUTTON_H_

#include <QtWidgets/QPushButton>
#include <GUI/Property/ToggleButton.h>

namespace GUI {
namespace Property {

class Qt5ToggleButton : public QObject, public ToggleButton {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5ToggleButton> Ptr;
		typedef std::weak_ptr<Qt5ToggleButton> WPtr;

	public:
		Qt5ToggleButton(std::string label);
		virtual ~Qt5ToggleButton();

		void show();
		void hide();
		bool visible() const;
		void enable();
		void disable();
		bool enabled() const;

		bool value() const;
		void setValue(bool value);

		void setLabel(std::string label);

		QWidget* widget();

	public slots:
		void buttonToggled(bool state);

	protected:
		QPushButton* m_button;
};

} // Property
} // GUI


#endif /* QT5TOGGLEBUTTON_H_ */
