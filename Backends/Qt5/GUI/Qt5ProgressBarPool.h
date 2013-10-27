/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5PROGRESSBARPOOL_H_
#define QT5PROGRESSBARPOOL_H_

#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>

#include <GUI/GUIProgressBarPool.h>
#include "Qt5ProgressBar.h"

namespace GUI {

class Qt5ProgressBarPool : public QObject, public ProgressBarPool {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5ProgressBarPool> Ptr;
		typedef std::weak_ptr<Qt5ProgressBarPool> WPtr;
		friend class Qt5ProgressBar;

	public:
		Qt5ProgressBarPool();
		virtual ~Qt5ProgressBarPool();

		void      show();
		void      hide();
		QWidget*  widget();

	protected:
		ProgressBar::Ptr createProgressBar(std::string label, int steps);
		void removeProgressBar(int index);

	protected:
		QWidget*                          m_area;
		QVBoxLayout*                      m_box;
		std::vector<Qt5ProgressBar::Ptr>  m_bars;
};

} // GUI

#endif /* QT5PROGRESSBARPOOL_H_ */
