#ifndef GUIMODEHANDLE_H_
#define GUIMODEHANDLE_H_

/**
 *  @file GUI/Mode/Handle.h
 *
 *  @brief Defines access handle to mode management
 */

#include <include/common.h>

#include "ModeGroup.h"
#include "../GUILog.h"

namespace GUI {
namespace Mode {


/**
 *  Access handle to mode management
 *
 *  This handle manages mode groups, which are closed
 *  groups of disjoint options (i.e. in each group at most
 *  one option can be active).
 *
 *  A possible backend implementation could implement
 *  modes as toolbar buttons grouped in according
 *  to the mode groups defined here.
 */
class Handle {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Handle> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Handle>   WPtr;

	public:
		/**
		 *  Constructor
		 *
		 *  @param log Access handle to log facility
		 */
		Handle(Log::Ptr log);

		/** Destructor */
		virtual ~Handle();

		/**
		 *  Adds group to GUI and returns a handle to it.
		 *
		 *  @param id Unique identifier of the group
		 *  @return Shared pointer to created group
		 */
		Group::Ptr addGroup(std::string id);

		/**
		 *  Removes specified group.
		 *
		 *  @param id Identifier of group to remove
		 */
		void removeGroup(std::string id);

		/**
		 *  Return specified group.
		 *
		 *  @param id Identifier of group to return
		 *  @return Shared pointer to group specified by id
		 */
		Group::Ptr group(std::string id);

		/**
		 *  Enable all groups.
		 *
		 *  Enabled groups are those groups the user can interact with.
		 *  By default groups are enabled.
		 */
		void enable();

		/**
		 *  Disable all groups.
		 *
		 *  Disabled groups are those groups the user cannot interact with.
		 *  By default groups are enabled.
		 */
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
