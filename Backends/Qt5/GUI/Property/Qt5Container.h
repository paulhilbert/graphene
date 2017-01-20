/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5CONTAINER_H_
#define QT5CONTAINER_H_

#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QGridLayout>

#include <include/visualizer.h>
#include <GUI/Property/PropContainer.h>

namespace GUI {
namespace Property {

class Qt5Container : public Container {
	public:
		typedef std::shared_ptr<Qt5Container> Ptr;
		typedef std::weak_ptr<Qt5Container>   WPtr;

	public:
		Qt5Container(bool scrollable = false);
		virtual ~Qt5Container();

		void setWidget(QWidget* widget);

		void show();
		void hide();
		bool visible() const;
		void enable();
		void disable();
		bool enabled() const;

		QWidget* widget();

		std::shared_ptr<Boolean>       createBool(std::string label);
		std::shared_ptr<Button>        createButton(std::string label);
		std::shared_ptr<Choice>        createChoice(std::string label);
		std::shared_ptr<Color>         createColor(std::string label);
		std::shared_ptr<File>          createFile(std::string label);
		std::shared_ptr<Files>         createFiles(std::string label);
		std::shared_ptr<Folder>        createFolder(std::string label);
		std::shared_ptr<Group>         createGroup(std::string label);
		std::shared_ptr<Number>        createNumber(std::string label);
		std::shared_ptr<Range>         createRange(std::string label);
		std::shared_ptr<Section>       createSection(std::string label);
		std::shared_ptr<String>        createString(std::string label);
		std::shared_ptr<ToggleButton>  createToggleButton(std::string label);
		std::shared_ptr<Tree>          createTree(std::string label);
		std::shared_ptr<Separator>     createSeparator();

	protected:
		bool          m_scrollable;
		QWidget*      m_outerArea;
		QVBoxLayout*  m_outerBox;
		QScrollArea*  m_scroll;
		QWidget*      m_area;
		QVBoxLayout*  m_box;
		QWidget*      m_widget;
		QGridLayout*  m_layout;
		unsigned int  m_widgetCount;
};


} // Property
} // GUI

#endif /* QT5CONTAINER_H_ */
