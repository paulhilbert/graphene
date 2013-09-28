#ifndef QT5CONTAINER_H_
#define QT5CONTAINER_H_

#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QGridLayout>

#include <GUI/Property/Bool.h>
#include <GUI/Property/Button.h>
#include <GUI/Property/Choice.h>
#include <GUI/Property/Color.h>
#include <GUI/Property/Container.h>
#include <GUI/Property/File.h>
#include <GUI/Property/Files.h>
#include <GUI/Property/Folder.h>
#include <GUI/Property/Group.h>
#include <GUI/Property/Number.h>
#include <GUI/Property/Range.h>
#include <GUI/Property/Section.h>
#include <GUI/Property/Separator.h>
#include <GUI/Property/String.h>
#include <GUI/Property/ToggleButton.h>
#include <GUI/Property/Tree.h>

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
		std::shared_ptr<ToggleButton>  createToggleButton(std::string label);
		std::shared_ptr<Tree>          createTree(std::string label);
		std::shared_ptr<Separator>     createSeparator();

	protected:
		bool          m_scrollable;
		QWidget*      m_outerArea     = nullptr;
		QVBoxLayout*  m_outerBox      = nullptr;
		QScrollArea*  m_scroll        = nullptr;
		QWidget*      m_area          = new QWidget();
		QVBoxLayout*  m_box           = new QVBoxLayout();
		QWidget*      m_widget        = nullptr;
		QGridLayout*  m_layout        = new QGridLayout();
		unsigned int  m_widgetCount;
};


} // Property
} // GUI

#endif /* QT5CONTAINER_H_ */
