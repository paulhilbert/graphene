#ifndef FWMODIFIER_H_
#define FWMODIFIER_H_

#include <memory>

namespace FW {
namespace Events {

class Modifier {
	public:
		typedef std::shared_ptr<Modifier> Ptr;
		typedef std::weak_ptr<Modifier>   WPtr;

	public:
		Modifier();
		virtual ~Modifier();

		bool& ctrl();
		bool& shift();
		bool& alt();
		bool& altgr();

	protected:
		bool m_ctrl;
		bool m_shift;
		bool m_alt;
		bool m_altgr;
};

} // Events
} // FW

#endif /* FWMODIFIER_H_ */
