#ifndef GUIMODEHANDLE_H_
#define GUIMODEHANDLE_H_

#include <iostream>
#include <memory>
#include <map>
#include <vector>

#include "ModeGroup.h"
#include "../GUILog.h"

namespace GUI {
namespace Mode {

class Handle {
	public:
		typedef std::shared_ptr<Handle> Ptr;
		typedef std::weak_ptr<Handle>   WPtr;

	public:
		Handle(Log::Ptr log);
		virtual ~Handle();

		Group::Ptr addGroup(std::string id);
		void removeGroup(std::string id);

		Group::Ptr group(std::string id);

		void enable();
		void disable();

	protected:
		virtual Group::Ptr createGroup() = 0;

	protected:
		Log::Ptr                          m_log;
		std::map<std::string, Group::Ptr> m_groups;
};

} // Mode
} // GUI

#endif /* GUIMODEHANDLE_H_ */
