/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5STATUS_H_
#define QT5STATUS_H_

#include <GUI/Status.h>
#include <QtWidgets/QMainWindow>

namespace GUI {

class Qt5Status : public Status {
	public:
		typedef std::shared_ptr<Qt5Status> Ptr;
		typedef std::weak_ptr<Qt5Status>   WPtr;

	public:
		Qt5Status(QMainWindow* mainWindow);
		virtual ~Qt5Status();

		void         set(const std::string& text);
		std::string  get() const;
		void         clear();

	protected:
		QMainWindow* m_mainWindow;
};


} // GUI

#endif /* QT5STATUS_H_ */
