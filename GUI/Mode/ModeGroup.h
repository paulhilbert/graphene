#ifndef GUIMODEGROUP_H_
#define GUIMODEGROUP_H_

/**
 *  @file ModeGroup.h
 *
 *  Defines handle class to mode groups.
 */

#include <include/common.h>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "Option.h"
#include "../GUILog.h"

namespace GUI {
namespace Mode {

/**
 *  Handle class to mode groups.
 *
 *  For a description see GUI::Mode::Handle class; for examples see example visualizers.
 */
class Group {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Group> Ptr;
		/** Weak pointer to this class */
		typedef std::weak_ptr<Group> WPtr;
		/** Function type to call on option change. Parameters are (mode_id, activeState) */
		typedef std::function<void (std::string, bool)> Callback;

	public:
		/**
		 *  @internal Group(Log::Ptr log)
		 *
		 *  @brief Constructor
		 *
		 *  @param log Access handle to log facility
		 */
		Group(Log::Ptr log);

		/** @internal ~Group()
		 *
		 *  @brief Destructor
		 */
		virtual ~Group();

		/**
		 *  Adds new option to this group.
		 *
		 *  @param id Identifier (unique in this group) of option
		 *  @param label Label for option (often used in tooltips)
		 *  @param icon Filesystem path to icon for this option
		 */
		Option::Ptr addOption(std::string id, std::string label, fs::path icon);

		/**
		 *  Removes specified option.
		 *
		 *  @param id Identifier of option to remove
		 */
		void removeOption(std::string id);

		/**
		 *  Set function to call when user changes active option.
		 *
		 *  @param onChange Callback function to call.
		 */
		void setCallback(Callback onChange);

		/**
		 *  Returns specified option.
		 *
		 *  @param id Identifier of option to return.
		 */
		Option::Ptr option(std::string id);

		/**
		 *  Returns identifier of currently active option.
		 *
		 *  @return Identifier of currently active option.
		 */
		std::string getCurrentOption() const;

		/**
		 *  Enable all options in this group.
		 *
		 *  Enabled options are those the user can interact with.
		 */
		void enable();

		/**
		 *  Disable all options in this group.
		 *
		 *  Disabled options are those the user cannot interact with.
		 *  Usually backends render disabled option grayed-out.
		 */
		void disable();

		/** Show all options in this group. By default all options are visible. */
		void show();

		/** Hide all options in this group. */
		void hide();

		/**
		 *  @internal notify(std::string modeId)
		 *
		 *  @brief Function called by backend implementation to notify about option changes.
		 *
		 *  @param modeId Identifier of option changing state.
		 */
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
