/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef PROPERTYCHOICE_H_
#define PROPERTYCHOICE_H_

/**
 *  @file Choice.h
 *
 *  Defines property type for choices.
 *
 */

#include <include/common.h>

#include "PropBase.h"
#include "PropNotify.h"
#include "PropValue.h"
#include "PropLabeled.h"

namespace GUI {
namespace Property {

/**
 *  Choice property type.
 *
 *  This property defines GUI functionality to enable choosing between predefined options.
 *
 *  Backends usually implement this property as a set of grouped radio buttons.
 */
class Choice : public Base, public Notify<void (std::string)>, public Value<std::string>, public Labeled {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Choice> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Choice>   WPtr;

		/** Specific callback type for this property */
		using Notify<void (std::string)>::Callback;

		/**
		 *  Type of single options to choose from.
		 */
		struct Option {
			/** Unique identifier of this option */
			std::string  id;
			/** Label of this option */
			std::string  label;
		};

	public:
		/**
		 *  @internal Choice(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		Choice(std::string label);

		/**
		 *  @internal ~Choice()
		 *
		 *  @brief Destructor
		 */
		virtual ~Choice();

		/**
		 *  Adds new option with given identifier and label.
		 *
		 *  @param id Unique identifier of the new option.
		 *  @param label GUI label to use for the new option.
		 */
		Choice& add(std::string id, std::string label);

		/**
		 *  Adds new option.
		 *
		 *  @param option Option object to add.
		 */
		Choice& add(const Option& option);

		/**
		 *  Adds multiple options.
		 *
		 *  @param options Vector of options to add.
		 */
		Choice& add(std::vector<Option>& options);
		
		/**
		 *  Returns identifier of currently active option.
		 *
		 *  @return Currently active option identifier.
		 */
		std::string value() const;

		/**
		 *  Sets currently active option.
		 *
		 *  @param id Identifier of option to set as currently active.
		 */
		void setValue(std::string id);

	protected:
		virtual void addOption(std::string label) = 0;
		virtual unsigned int getActiveOption() const = 0;
		virtual void setActiveOption(unsigned int option) = 0;

	protected:
		std::vector<std::string>  m_options;
};


} // Property
} // GUI


#endif /* PROPERTYCHOICE_H_ */
