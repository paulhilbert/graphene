#ifndef RAW_ARRAY_2D_H
#define RAW_ARRAY_2D_H

#include <cstring>

#include <fstream>
using std::ofstream;
using std::ifstream;
using std::ios;

namespace Data {

template <class T>
class RawArray2D {
protected:
	typedef unsigned int uint;

public:
	RawArray2D();
	RawArray2D(uint rows, uint cols);
	RawArray2D(uint rows, uint cols, T val);
	RawArray2D(const RawArray2D<T>& other);
	virtual ~RawArray2D();

	RawArray2D<T>& operator=(const RawArray2D<T>& other);
	T& operator()(uint row, uint col);
	const T& operator()(uint row, uint col) const;

	inline T*   getData()     {return m_data;}
	inline uint getRowCount() {return m_rows;}
	inline uint getColCount() {return m_cols;}

	void setRange(uint rowMin, uint rowMax, uint colMin, uint colMax, T val);

	void serializeBinary(string file) const;
	void deserializeBinary(string file);

protected:
	T*   m_data;
	uint m_rows;
	uint m_cols;

protected:
	void init(uint rows, uint cols);
};

// Implementation

template <class T>
RawArray2D<T>::RawArray2D() {
	m_data = NULL;
	m_rows = m_cols = 0;
}

template <class T>
RawArray2D<T>::RawArray2D(uint rows, uint cols) {
	m_data = NULL;
	init(rows, cols);
}

template <class T>
RawArray2D<T>::RawArray2D(uint rows, uint cols, T val) {
	m_data = NULL;
	init(rows, cols);
	setRange(0, m_rows-1, 0, m_cols-1, val);
}

template <class T>
RawArray2D<T>::RawArray2D(const RawArray2D<T>& other) {
	m_data = NULL;
	init(other.m_rows, other.m_cols);
	operator=(other);
}

template <class T>
RawArray2D<T>::~RawArray2D() {
	if (m_data) delete[] m_data;
}

template <class T>
inline RawArray2D<T>& RawArray2D<T>::operator=(const RawArray2D<T>& other) {
	memcpy(m_data, other.m_data, m_rows*m_cols*sizeof(T));
	return *this;
}

template <class T>
inline T& RawArray2D<T>::operator()(uint row, uint col) {
	return m_data[row*m_cols+col];
}

template <class T>
inline const T& RawArray2D<T>::operator()(uint row, uint col) const {
	return m_data[row*m_cols+col];
}

template <class T>
inline void RawArray2D<T>::setRange(uint rowMin, uint rowMax, uint colMin, uint colMax, T val) {
	for (uint row = rowMin; row < rowMax; ++row) {
		for (uint col = colMin; col < colMax; ++col) {
			m_data[row*m_cols+col] = val;
		}
	}
}

template <class T>
inline void RawArray2D<T>::init(uint rows, uint cols) {
	if (m_data) delete[] m_data;
	m_rows = rows;
	m_cols = cols;
	m_data = new T[m_rows*m_cols];
}

template <class T>
void RawArray2D<T>::serializeBinary(string file) const {
	ofstream out(file.c_str(), ios::out | ios::binary);
	out.write((char*) &m_rows, sizeof (uint));
	out.write((char*) &m_cols, sizeof (uint));
	out.write((char*)m_data, m_rows * m_cols * sizeof (T));
	out.close();
}

template <class T>
void RawArray2D<T>::deserializeBinary(string file) {
	if (m_data) delete [] m_data;
	ifstream in(file.c_str(), ios::in | ios::binary);
	in.read((char*) &m_rows, sizeof (uint));
	in.read((char*) &m_cols, sizeof (uint));
	m_data = new T[m_rows*m_cols];
	in.read((char*)m_data, m_rows * m_cols * sizeof (T));
	in.close();
}


}

#endif
