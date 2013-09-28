#ifndef GUIMODEOPTION_H_
#define GUIMODEOPTION_H_

#include <iostream>
#include <memory>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "../GUIElement.h"

namespace GUI {
namespace Mode {

class Group;

class Option : public GUIElement {
	public:
		typedef std::shared_ptr<Option> Ptr;
		typedef std::weak_ptr<Option>   WPtr;

	public:
		Option(std::string id, std::string label, fs::path icon);
		virtual ~Option();

		virtual bool active() const = 0;
		virtual void setActive(bool active) = 0;

	protected:
		std::string m_id;
		std::string m_label;
		fs::path    m_icon;

};

} // Mode
} // GUI

#endif /* GUIMODEOPTION_H_ */
