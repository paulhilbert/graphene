/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5PROGRESSBARPOOL_H_
#define QT5PROGRESSBARPOOL_H_

#include <mutex>
#include <map>

#include <QtCore/QSignalMapper>
#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>

#include <GUI/GUIProgressBarPool.h>
#include "Qt5ProgressBar.h"

namespace GUI {

class Qt5ProgressBarPool : public QObject {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5ProgressBarPool>  Ptr;
		typedef std::weak_ptr<Qt5ProgressBarPool>    WPtr;

	public:
		Qt5ProgressBarPool();
		virtual ~Qt5ProgressBarPool();

		void      show();
		void      hide();
		QWidget*  widget();

		void  setBarCountChangeCallback(std::function<void (int)> callback);

	public slots:
		void create(QString label, int steps);
		void removeProgressBar(int index);
		void poll(int index, float progress);

	signals:
		void sendBar(ProgressBar::Ptr bar);

	protected:
		QWidget* m_area;
		QVBoxLayout* m_box;
		std::map<int, Qt5ProgressBar::Ptr>  m_bars;
		std::mutex                          m_mutex;
		//QSignalMapper*                      m_mapper;
		int                                 m_lastIndex;
		std::function<void (int)>           m_callback;
};

} // GUI

#endif /* QT5PROGRESSBARPOOL_H_ */
