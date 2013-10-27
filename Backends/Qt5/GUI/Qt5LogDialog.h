/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5LOGDIALOG_H_
#define QT5LOGDIALOG_H_

#include <include/common.h>

#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QTextEdit>
#include <QtCore/QString>

#include "Qt5ProgressBarPool.h"

namespace GUI {

class Qt5LogDialog : public QDockWidget {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5LogDialog> Ptr;
		typedef std::weak_ptr<Qt5LogDialog> WPtr;

	public:
		Qt5LogDialog(std::string title);
		virtual ~Qt5LogDialog();

		void logInfo(std::string text);
		void logWarn(std::string text);
		void logError(std::string text);
		void logVerbose(std::string text);
		void clear();

		ProgressBarPool::Ptr progressBarPool();

	protected:
		QString format(const std::string& text);
		QString formatPrefix(const std::string& text, const std::string& color);
		void append(const QString& string);
	
	protected:
		QWidget*                 m_area;
		QVBoxLayout*             m_box;
		QTextEdit*               m_text;
		Qt5ProgressBarPool::Ptr  m_barPool;

};

} // GUI

#endif /* QT5LOGDIALOG_H_ */
