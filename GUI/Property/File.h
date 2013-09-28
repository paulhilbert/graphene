#ifndef PROPERTYFILE_H_
#define PROPERTYFILE_H_

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "Base.h"
#include "Notify.h"
#include "Value.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

class File : public Base, public Notify<fs::path>, public Value<fs::path>, public Labeled {
	public:
		typedef std::shared_ptr<File> Ptr;
		typedef std::weak_ptr<File>   WPtr;
		using Notify<fs::path>::Callback;

		typedef enum {OPEN, SAVE} Mode;

	public:
		File(std::string label);
		virtual ~File();

		virtual void setMode(Mode mode) = 0;
		virtual void setExtensions(std::vector<std::string> extensions) = 0;
};

} // Property
} // GUI

#endif /* PROPERTYFILE_H_ */
