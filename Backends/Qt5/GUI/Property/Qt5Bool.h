#ifndef QT5BOOL_H_
#define QT5BOOL_H_

#include <QtWidgets/QLabel>
#include <QtWidgets/QCheckBox>
#include <GUI/Property/Bool.h>

namespace GUI {
namespace Property {

class Qt5Bool : public QObject, public Bool {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5Bool> Ptr;
		typedef std::weak_ptr<Qt5Bool> WPtr;

	public:
		Qt5Bool(std::string label, QLabel* labelWidget);
		virtual ~Qt5Bool();

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
		void checkboxClicked(int state);

	protected:
		QLabel*    m_labelWidget;
		QCheckBox* m_checkBox;
};


} // Property
} // GUI

#endif /* QT5BOOL_H_ */
