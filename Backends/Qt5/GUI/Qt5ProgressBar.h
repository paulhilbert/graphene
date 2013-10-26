/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5PROGRESSBAR_H_
#define QT5PROGRESSBAR_H_

#include <include/common.h>

#include <QtWidgets/QWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QProgressBar>

namespace GUI {

class Qt5ProgressBar : public QObject {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5ProgressBar> Ptr;
		typedef std::weak_ptr<Qt5ProgressBar> WPtr;

	public:
		Qt5ProgressBar(std::string label);
		virtual ~Qt5ProgressBar();

		QWidget* widget();

	public slots:
		void poll(float progress);

	protected:
		std::string   m_label;
		QWidget*      m_area;
		QHBoxLayout*  m_box;
		QLabel*       m_labelWidget;
		QProgressBar* m_bar;
};


} // GUI

#endif /* QT5PROGRESSBAR_H_ */
