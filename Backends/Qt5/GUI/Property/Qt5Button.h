#ifndef QT5BUTTON_H_
#define QT5BUTTON_H_

#include <QtWidgets/QPushButton>
#include <GUI/Property/Button.h>

namespace GUI {
namespace Property {

class Qt5Button : public QObject, public Button {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5Button> Ptr;
		typedef std::weak_ptr<Qt5Button> WPtr;

	public:
		Qt5Button(std::string label);
		virtual ~Qt5Button();

		void show();
		void hide();
		bool visible() const;
		void enable();
		void disable();
		bool enabled() const;

		void setLabel(std::string label);

		QWidget* widget();

	public slots:
		void buttonClicked();

	protected:
		QPushButton* m_button;
};


} // Property
} // GUI

#endif /* QT5BUTTON_H_ */
