#ifndef INDENTATION_H_
#define INDENTATION_H_

#include <iostream>
using std::string;

namespace StringUtils {

class Indentation {
	public:
		Indentation (int depth = 0, int width = 3) : m_depth(depth), m_width(width) {}
		virtual ~Indentation() {}

		Indentation(const Indentation& other) {
			m_depth = other.getDepth();
			m_width = other.getWidth();
		}

		inline int getDepth() const { return m_depth; }
		inline int getWidth() const { return m_width; }
		inline void setDepth(int depth) { m_depth = depth; }
		inline void setWidth(int width) { m_width = width; }

		inline void increaseDepth() { ++m_depth; }
		inline void decreaseDepth() { --m_depth; }

		inline string getIndentation() const {
			string indent("");
			string indentation("");
			for (int i = 0; i < m_width; ++i) indent += " ";
			for (int i = 0; i < m_depth; ++i) indentation += indent;

			return indentation;
		}

		inline string getIndentedString(const string& input) const {
			return getIndentation() + input;
		}
		inline void indent(string& str) const {
			str = str + getIndentation();
		}

		static string getIndentedString(const string& input, int depth, int width = 0) {
			string indent("");
			string indentation("");
			for (int i = 0; i < width; ++i) indent += " ";
			for (int i = 0; i < depth; ++i) indentation += indent;

			return indentation + input;
		}
		static void indent(string& str, int depth = 0, int width = 0) {
			string indent("");
			string indentation("");
			for (int i = 0; i < width; ++i) indent += " ";
			for (int i = 0; i < depth; ++i) indentation += indent;

			str = indentation + str;
		}


	private:
		int m_depth;
		int m_width;
};


} // StringUtils

#endif /* INDENTATION_H_ */
