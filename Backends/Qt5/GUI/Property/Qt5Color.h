/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5COLOR_H_
#define QT5COLOR_H_

#include <QtWidgets/QWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <GUI/Property/Color.h>

namespace GUI {
namespace Property {

class Qt5Color : public QObject, public Color {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5Color> Ptr;
		typedef std::weak_ptr<Qt5Color>   WPtr;

	public:
		Qt5Color(std::string label, QLabel* labelWidget);
		virtual ~Qt5Color();

		void show();
		void hide();
		bool visible() const;
		void enable();
		void disable();
		bool enabled() const;

		Eigen::Vector4f value() const;
		void setValue(Eigen::Vector4f value);

		void setLabel(std::string label);

		QWidget* widget();

	protected:
		void setButtonColor();

	public slots:
		void buttonClicked();

	protected:
		QLabel*         m_labelWidget;
		QPushButton*    m_button;
		Eigen::Vector4f m_color;
};


} // Property
} // GUI

#endif /* QT5COLOR_H_ */
