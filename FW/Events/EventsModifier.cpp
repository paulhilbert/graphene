/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "EventsModifier.h"

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
