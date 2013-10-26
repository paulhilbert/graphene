/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Qt5Tree.h"

#include <include/common.h>
#include <QtWidgets/QHeaderView>

namespace GUI {
namespace Property {

Qt5Tree::Qt5Tree(std::string label) : Tree(label), m_area(new QWidget()), m_vBox(new QVBoxLayout()), m_labelWidget(new QLabel(QString::fromStdString(label))), m_tree(new QTreeWidget()) {
	m_area->setLayout(m_vBox);
	m_vBox->addWidget(m_labelWidget, 0);
	m_vBox->addWidget(m_tree, 1);
	m_tree->setColumnCount(1);
	m_tree->header()->close();
	QObject::connect(m_tree, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this, SLOT(itemChanged(QTreeWidgetItem*, int)));
}

Qt5Tree::~Qt5Tree() {
}

void Qt5Tree::removePath(const std::vector<std::string>& path) {
	auto item = getItem(path, nullptr);
	if (item) delete item;
}

void Qt5Tree::show() {
	m_area->show();
}

void Qt5Tree::hide() {
	m_area->hide();
}

bool Qt5Tree::visible() const {
	return m_area->isVisible();
}

void Qt5Tree::enable() {
	m_area->setEnabled(true);
}

void Qt5Tree::disable() {
	m_area->setEnabled(false);
}

bool Qt5Tree::enabled() const {
	return m_area->isEnabled();
}

void Qt5Tree::setLabel(std::string label) {
	m_labelWidget->setText(QString::fromStdString(label));
}

QWidget* Qt5Tree::widget() {
	return m_area;
}

void Qt5Tree::itemChanged(QTreeWidgetItem* item, int) {
	QTreeWidgetItem* crt = item;
	std::vector<std::string> path(1, item->text(0).toStdString());
	while ((crt = crt->parent())) {
		path.push_back(crt->text(0).toStdString());
	}
	std::reverse(path.begin(), path.end());
	notify(getId(path), item->checkState(0) == Qt::Checked);
}

void Qt5Tree::add(const std::vector<std::string>& path, bool checked) {
	asserts(path.size(), "Empty path given");
	QTreeWidgetItem* newItem = new QTreeWidgetItem(QStringList(QString::fromStdString(path.back())));
	newItem->setCheckState(0, checked ? Qt::Checked : Qt::Unchecked);
	if (path.size() == 1) {
		m_tree->addTopLevelItem(newItem);
	} else {
		std::vector<std::string>::const_iterator prev = path.end();
		std::advance(prev, -1);
		std::vector<std::string> pathSuffix(path.begin(), prev);
		auto item = getItem(pathSuffix, nullptr);
		if (!item) return;
		item->addChild(newItem);
		item->setExpanded(true);
	}
}

QTreeWidgetItem* Qt5Tree::getItem(const std::vector<std::string>& path, QTreeWidgetItem* parent) {
	QTreeWidgetItem* first = nullptr;
	QString firstStr = QString::fromStdString(path.front());
	if (!parent) {
		int count = m_tree->topLevelItemCount();
		for (int i=0; i<count; ++i) {
			QTreeWidgetItem* candidate = m_tree->topLevelItem(i);
			if (candidate->text(0) == firstStr) {
				first = candidate;
				break;
			}
		}
	} else {
		int count = parent->childCount();
		for (int i=0; i<count; ++i) {
			QTreeWidgetItem* candidate = parent->child(i);
			if (candidate->text(0) == firstStr) {
				first = candidate;
				break;
			}
		}
	}
	asserts(first, "Incorrect path for Tree property");
	if (path.size() == 1) return first;
	std::vector<std::string>::const_iterator snd = path.begin();
	std::advance(snd, 1);
	std::vector<std::string> rest(snd, path.end());
	return getItem(rest, first);
}


} // Property
} // GUI
