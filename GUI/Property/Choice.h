#ifndef PROPERTYCHOICE_H_
#define PROPERTYCHOICE_H_

#include <vector>

#include "Base.h"
#include "Notify.h"
#include "Value.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

class Choice : public Base, public Notify<void (std::string)>, public Value<std::string>, public Labeled {
	public:
		typedef std::shared_ptr<Choice> Ptr;
		typedef std::weak_ptr<Choice>   WPtr;
		using Notify<void (std::string)>::Callback;

		struct Option {
			std::string  id;
			std::string  label;
		};

	public:
		Choice(std::string label);
		virtual ~Choice();

		void add(std::string id, std::string label);
		void add(const Option& option);
		void add(std::vector<Option>& options);
		
		std::string value() const;
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
