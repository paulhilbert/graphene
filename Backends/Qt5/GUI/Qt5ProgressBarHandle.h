/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5PROGRESSBARHANDLE_H_
#define QT5PROGRESSBARHANDLE_H_

#include <QtCore/QObject>

#include <GUI/GUIProgressBar.h>
#include <GUI/GUIProgressBarPool.h>

namespace GUI {

class Qt5ProgressBarHandle : public QObject, public ProgressBar {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5ProgressBarHandle> Ptr;
		typedef std::weak_ptr<Qt5ProgressBarHandle> WPtr;

	public:
		Qt5ProgressBarHandle(int idx, std::string label, int steps = 1);
		virtual ~Qt5ProgressBarHandle();

		void poll(float progress);

	signals:
		void polled(int idx, float progress);
		void sigDestroy(int idx);

	protected:
		int m_idx;
};


} // GUI

#endif /* QT5PROGRESSBARHANDLE_H_ */
