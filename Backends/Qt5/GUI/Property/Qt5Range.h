/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5RANGE_H_
#define QT5RANGE_H_

#include <QtWidgets/QWidget>
#include <QtWidgets/QLabel>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QSlider>
#include <GUI/Property/Range.h>

namespace GUI {
namespace Property {

class Qt5Range : public QObject, public Range {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5Range> Ptr;
		typedef std::weak_ptr<Qt5Range>   WPtr;

	public:
		Qt5Range(std::string label, QLabel* labelWidget);
		virtual ~Qt5Range();

		void setMin(double min);
		void setMax(double max);
		void setDigits(int digits);

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

	protected:
		void updateSlider();
		void updateLabel(double value);

	public slots:
		void valueChanged(int value);

	protected:
		QLabel*      m_labelWidget;
		QWidget*     m_area;
		QHBoxLayout* m_box;
		QLabel*      m_valueLabel;
		QSlider*     m_slider;
		double       m_minimum;
		double       m_maximum;
		int          m_digits;
};

} // Property
} // GUI

#endif /* QT5RANGE_H_ */
