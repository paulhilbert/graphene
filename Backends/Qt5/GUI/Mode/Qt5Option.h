/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5OPTION_H_
#define QT5OPTION_H_

#include <GUI/Mode/Option.h>
#include <QtWidgets/QToolButton>

namespace GUI {
namespace Mode {

class Qt5Group;

class Qt5Option : public QObject, public Option {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5Option> Ptr;
		typedef std::weak_ptr<Qt5Option>   WPtr;
		friend class Qt5Group;

	public:
		Qt5Option(std::string id, std::string label, fs::path icon, Qt5Group* group);
		virtual ~Qt5Option();

		bool active() const;
		void setActive(bool active);

		void show();
		void hide();
		bool visible() const;

		void enable();
		void disable();
		bool enabled() const;

		QWidget* widget();

	public slots:
		void stateChanged();

	protected:
		Qt5Group*    m_group;
		QToolButton* m_button;
};


} // Mode
} // GUI

#endif /* QT5OPTION_H_ */
