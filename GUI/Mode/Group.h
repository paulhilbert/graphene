#ifndef GUIMODEGROUP_H_
#define GUIMODEGROUP_H_

#include <iostream>
#include <memory>
#include <functional>
#include <map>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "Option.h"
#include "../Log.h"

namespace GUI {
namespace Mode {

class Group {
	public:
		typedef std::shared_ptr<Group> Ptr;
		typedef std::weak_ptr<Group> WPtr;
		typedef std::function<void (std::string)> Callback;

	public:
		Group(Log::Ptr log, const Callback& onChange = nullptr);
		virtual ~Group();

		Option::Ptr addOption(std::string id, std::string label, fs::path icon);
		void removeOption(std::string id);

		Option::Ptr mode(std::string id);
		std::string getCurrentOption() const; // returns id

		void enable();
		void disable();
		void show();
		void hide();

		void notify(std::string modeId); // for internal use

	protected:
		virtual Option::Ptr createOption(std::string id, std::string label, fs::path icon) = 0;

	protected:
		Log::Ptr                           m_log;
		Callback                           m_onChange;
		std::map<std::string, Option::Ptr> m_modes;
		std::map<std::string, bool>        m_enabledStates;
		std::map<std::string, bool>        m_visibleStates;
};

} // Mode
} // GUI

#endif /* GUIMODEGROUP_H_ */
