#ifndef QT5TREE_H_
#define QT5TREE_H_

#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QTreeWidget>
#include <GUI/Property/Tree.h>

namespace GUI {
namespace Property {

class Qt5Tree : public QObject, public Tree {
	Q_OBJECT

	public:
		typedef std::shared_ptr<Qt5Tree> Ptr;
		typedef std::weak_ptr<Qt5Tree> WPtr;

	public:
		Qt5Tree(std::string label);
		virtual ~Qt5Tree();

		void removePath(const std::vector<std::string>& path);

		void show();
		void hide();
		bool visible() const;
		void enable();
		void disable();
		bool enabled() const;

		void setLabel(std::string label);

		QWidget* widget();

	public slots:
		void itemChanged(QTreeWidgetItem* item, int);

	protected:
		void add(const std::vector<std::string>& path, bool checked);
		QTreeWidgetItem* getItem(const std::vector<std::string>& path, QTreeWidgetItem* parent);

	protected:
		QWidget*      m_area;
		QVBoxLayout*  m_vBox;
		QLabel*       m_labelWidget;
		QTreeWidget*  m_tree;
};


} // Property
} // GUI

#endif /* QT5TREE_H_ */
