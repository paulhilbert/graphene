/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5FILE_H_
#define QT5FILE_H_

#include <QtWidgets/QWidget>
#include <QtWidgets/QLabel>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QLineEdit>
#include <GUI/Property/PropFile.h>

namespace GUI {
namespace Property {

class Qt5File : public QObject, public File {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5File> Ptr;
		typedef std::weak_ptr<Qt5File>   WPtr;

	public:
		Qt5File(std::string label);
		virtual ~Qt5File();

		void setMode(Mode mode);
		void setExtensions(std::vector<std::string> extensions);

		void show();
		void hide();
		bool visible() const;
		void enable();
		void disable();
		bool enabled() const;

		fs::path value() const;
		void setValue(fs::path value);

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
		Mode          m_mode;
		std::vector<std::string> m_extensions;
};


} // Property
} // GUI

#endif /* QT5FILE_H_ */
