/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5NUMBER_H_
#define QT5NUMBER_H_

#include <QtWidgets/QWidget>
#include <QtWidgets/QLabel>
#include <QtWidgets/QDoubleSpinBox>
#include <GUI/Property/PropNumber.h>

namespace GUI {
namespace Property {

class Qt5Number : public QObject, public Number {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5Number> Ptr;
		typedef std::weak_ptr<Qt5Number>   WPtr;

	public:
		Qt5Number(std::string label, QLabel* labelWidget);
		virtual ~Qt5Number();

		Number& setMin(double min);
		Number& setMax(double max);
		Number& setDigits(int digits);
		Number& setStep(double step);

		void show();
		void hide();
		bool visible() const;
		void enable();
		void disable();
		bool enabled() const;

		double value() const;
		void setValue(double value);

		void setLabel(std::string label);

		QWidget* widget();

	public slots:
		void valueChanged(double value);

	protected:
		QLabel*          m_labelWidget;
		QDoubleSpinBox*  m_spinBox;
};


} // Property
} // GUI

#endif /* QT5NUMBER_H_ */
