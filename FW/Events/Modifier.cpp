#include "Modifier.h"

namespace FW {
namespace Events {

Modifier::Modifier() : m_ctrl(false), m_shift(false), m_alt(false), m_altgr(false) {
}

Modifier::~Modifier() {
}

bool& Modifier::ctrl() {
	return m_ctrl;
}

bool& Modifier::shift() {
	return m_shift;
}

bool& Modifier::alt() {
	return m_alt;
}

bool& Modifier::altgr() {
	return m_altgr;
}

} // Events
} // FW
