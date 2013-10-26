/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "VAO.h"

namespace Buffer {

VAO::VAO() : m_id(0) {
}

VAO::~VAO() {
	if (m_id) glDeleteVertexArrays(1, &m_id);
	m_id = 0;
}

void VAO::init() {
	glGenVertexArrays(1, &m_id);
}

} // Buffer
