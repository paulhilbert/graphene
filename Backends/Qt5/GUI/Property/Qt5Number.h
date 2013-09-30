#ifndef QT5NUMBER_H_
#define QT5NUMBER_H_

#include <QtWidgets/QWidget>
#include <QtWidgets/QLabel>
#include <QtWidgets/QDoubleSpinBox>
#include <GUI/Property/Number.h>

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

		void setMin(double min);
		void setMax(double max);
		void setDigits(int digits);
		void setStep(double step);

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
