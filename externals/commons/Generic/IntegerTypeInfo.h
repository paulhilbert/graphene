#ifndef INTEGERTYPEINFO_H_
#define INTEGERTYPEINFO_H_

#include <climits>
#include <typeinfo>

namespace Generic {

template <class Type>
class IntegerTypeInfo {
	public:
		IntegerTypeInfo ( );

		inline long getMinimumValue() const { return m_minimumValue; }
		inline unsigned long getMaximumValue() const { return m_maximumValue; }
		inline int getNumBits() const { return m_numBits; }

		inline operator bool() const { return m_valid; }
	
	protected:
		bool m_valid;
		long m_minimumValue;
		unsigned long m_maximumValue;
		int m_numBits;
};

template <class Type>
IntegerTypeInfo<Type>::IntegerTypeInfo ( ) {
	m_valid = true;
	m_numBits = sizeof(Type) * CHAR_BIT;
	if (typeid(Type) == typeid(signed char)) {
		m_minimumValue = static_cast<long>(SCHAR_MIN);
		m_maximumValue = static_cast<unsigned long>(SCHAR_MAX);
	}
	else if (typeid(Type) == typeid(unsigned char)) {
		m_minimumValue = 0;
		m_maximumValue = static_cast<unsigned long>(UCHAR_MAX);
	}
	else if (typeid(Type) == typeid(char)) {
		m_minimumValue = static_cast<long>(CHAR_MIN);
		m_maximumValue = static_cast<unsigned long>(CHAR_MAX);
	}
	else if (typeid(Type) == typeid(signed short)) {
		m_minimumValue = static_cast<long>(SHRT_MIN);
		m_maximumValue = static_cast<unsigned long>(SHRT_MAX);
	}
	else if (typeid(Type) == typeid(unsigned short)) {
		m_minimumValue = 0;
		m_maximumValue = static_cast<unsigned long>(USHRT_MAX);
	}
	else if (typeid(Type) == typeid(signed int)) {
		m_minimumValue = static_cast<long>(INT_MIN);
		m_maximumValue = static_cast<unsigned long>(INT_MAX);
	}
	else if (typeid(Type) == typeid(unsigned int)) {
		m_minimumValue = 0;
		m_maximumValue = static_cast<unsigned long>(UINT_MAX);
	}
	else if (typeid(Type) == typeid(signed long)) {
		m_minimumValue = static_cast<long>(LONG_MIN);
		m_maximumValue = static_cast<unsigned long>(LONG_MAX);
	}
	else if (typeid(Type) == typeid(unsigned long)) {
		m_minimumValue = 0;
		m_maximumValue = static_cast<unsigned long>(ULONG_MAX);
	}
	else { m_valid = false; }
}

} // Generic

#endif /* INTEGERTYPEINFO_H_ */
