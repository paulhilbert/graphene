#ifndef PROPERTYFOLDER_H_
#define PROPERTYFOLDER_H_

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "Base.h"
#include "Notify.h"
#include "Value.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

class Folder : public Base, public Notify<void (fs::path)>, public Value<fs::path>, public Labeled {
	public:
		typedef std::shared_ptr<Folder> Ptr;
		typedef std::weak_ptr<Folder>   WPtr;
		using Notify<void (fs::path)>::Callback;

	public:
		Folder(std::string label);
		virtual ~Folder();
};

} // Property
} // GUI

#endif /* PROPERTYFOLDER_H_ */
