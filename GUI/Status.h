#ifndef GUISTATUS_H_
#define GUISTATUS_H_

#include <string>
#include <memory>

namespace GUI {

class Status {
	public:
		typedef std::shared_ptr<Status> Ptr;
		typedef std::weak_ptr<Status>   WPtr;

	public:
		Status();
		virtual ~Status();

		virtual void         set(const std::string& text) = 0;
		virtual std::string  get() const = 0;
		virtual void         clear() = 0;
};

} // GUI

#endif /* GUISTATUS_H_ */
