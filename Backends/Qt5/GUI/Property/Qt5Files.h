#ifndef QT5FILES_H_
#define QT5FILES_H_

#include <QtWidgets/QWidget>
#include <QtWidgets/QLabel>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QLineEdit>
#include <GUI/Property/Files.h>

namespace GUI {
namespace Property {

class Qt5Files : public QObject, public Files {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5Files> Ptr;
		typedef std::weak_ptr<Qt5Files>   WPtr;

	public:
		Qt5Files(std::string label);
		virtual ~Qt5Files();

		void show();
		void hide();
		bool visible() const;
		void enable();
		void disable();
		bool enabled() const;

		Paths value() const;
		void setValue(Paths value);

		void setLabel(std::string label);

		QWidget* widget();

	public slots:
		void buttonClicked();

	protected:
		QWidget*      m_outer;
		QWidget*      m_inner;
		QVBoxLayout*  m_vBox;
		QHBoxLayout*  m_hBox;
		QLabel*       m_labelWidget;
		QLineEdit*    m_lineEdit;
		QPushButton*  m_button;
		Paths         m_value;
};


} // Property
} // GUI

#endif /* QT5FILES_H_ */
