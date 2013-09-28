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
