/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5STRING_H_
#define QT5STRING_H_

#include <QtWidgets/QWidget>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <GUI/Property/PropString.h>

namespace GUI {
namespace Property {

class Qt5String : public QObject, public String {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5String> Ptr;
		typedef std::weak_ptr<Qt5String>   WPtr;

	public:
		Qt5String(std::string label, QLabel* labelWidget);
		virtual ~Qt5String();

		void show();
		void hide();
		bool visible() const;
		void enable();
		void disable();
		bool enabled() const;

		std::string value() const;
		void setValue(std::string value);

		void setLabel(std::string label);

		QWidget* widget();

	public slots:
		void stringChanged(QString text);

	protected:
		QLabel*       m_labelWidget;
		QLineEdit*    m_lineEdit;
};

} // Property
} // GUI

#endif /* QT5STRING_H_ */
