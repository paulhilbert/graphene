#ifndef PROPERTYFILES_H_
#define PROPERTYFILES_H_

#include <vector>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "Base.h"
#include "Notify.h"
#include "Value.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

typedef std::vector<fs::path> Paths;

class Files : public Base, public Notify<void (Paths)>, public Value<Paths>, public Labeled {
	public:
		typedef std::shared_ptr<Files> Ptr;
		typedef std::weak_ptr<Files>   WPtr;
		using Notify<void (Paths)>::Callback;

	public:
		Files(std::string label);
		virtual ~Files();
};

} // Property
} // GUI

#endif /* PROPERTYFILES_H_ */
