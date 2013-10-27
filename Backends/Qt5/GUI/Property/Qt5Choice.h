/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5CHOICE_H_
#define QT5CHOICE_H_

#include <QtCore/QSignalMapper>
#include <QtWidgets/QWidget>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QRadioButton>
#include <GUI/Property/Choice.h>

namespace GUI {
namespace Property {

class Qt5Choice : public QObject, public Choice {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5Choice> Ptr;
		typedef std::weak_ptr<Qt5Choice> WPtr;

	public:
		Qt5Choice(std::string label);
		virtual ~Qt5Choice();

		void show();
		void hide();
		bool visible() const;
		void enable();
		void disable();
		bool enabled() const;

		void setLabel(std::string label);

		QWidget* widget();

	public slots:
		void optionChanged(int option);
	
	protected:
		void addOption(std::string label);
		unsigned int getActiveOption() const;
		void setActiveOption(unsigned int option);

	protected:
		QGroupBox*                  m_group;
		QVBoxLayout*                m_box;
		QSignalMapper*              m_signalMapper;
		std::vector<QRadioButton*>  m_radios;
		unsigned int                m_radioCount;
};


} // Property
} // GUI

#endif /* QT5CHOICE_H_ */
