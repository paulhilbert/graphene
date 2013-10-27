/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef QT5SECTION_H_
#define QT5SECTION_H_

#include <GUI/Property/Section.h>
#include "Qt5Container.h"
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>

namespace GUI {
namespace Property {

class Qt5Section : public QObject, public Section {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5Section> Ptr;
		typedef std::weak_ptr<Qt5Section>   WPtr;

	public:
		Qt5Section(std::string label);
		virtual ~Qt5Section();

		void setCollapsed(bool collapsed);

		void show();
		void hide();
		bool visible() const;
		void enable();
		void disable();
		bool enabled() const;

		void setLabel(std::string label);

		QWidget* widget();

	public slots:
		void toggled(bool state);

	protected:
		std::shared_ptr<Bool>          createBool(std::string label);
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
		std::shared_ptr<Tree>          createTree(std::string label);
		std::shared_ptr<ToggleButton>  createToggleButton(std::string label);
		std::shared_ptr<Separator>     createSeparator();

	protected:
		Qt5Container m_container;
		QGroupBox*   m_group;
		QWidget*     m_area;
		QVBoxLayout* m_box;
};

} // Property
} // GUI

#endif /* QT5SECTION_H_ */
