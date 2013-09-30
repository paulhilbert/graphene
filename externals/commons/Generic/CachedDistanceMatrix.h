#ifndef CACHEDDISTANCEMATRIX_H_
#define CACHEDDISTANCEMATRIX_H_

#include <cassert>
#include <limits>
using std::numeric_limits;

#include "CachedData.h"

namespace Generic {

template <class DataType, class KeyType>
class CachedDistanceMatrix : public CachedData<DataType,KeyType> {
	public:
		typedef boost::function<DataType (KeyType)> RetrievalFunc;
		typedef typename CachedData<DataType,KeyType>::Data Data;
		typedef typename CachedData<DataType,KeyType>::DataIter DataIter;
		typedef typename CachedData<DataType,KeyType>::DataConstIter DataConstIter;

	public:
		CachedDistanceMatrix(unsigned int dim, RetrievalFunc retrievalFunc);
		virtual ~CachedDistanceMatrix();

		DataType* getDataMatrix();
		const DataType* getDataMatrix() const;
		
		DataType get(KeyType key);
		bool drop(KeyType key);
		bool isCached(KeyType key) const;

#ifdef DEBUG
		float cachedQueries() const { return static_cast<float>(m_numCachedQueries) / m_totalQueries; }
		void resetCachedQueriesCounter() { m_numCachedQueries = 0; m_totalQueries = 0; }
#endif

	private:
		Data& getData();
		const Data& getData() const;
		DataIter getDataIterator(KeyType key);
		DataConstIter getDataIterator(KeyType key) const;

		DataType* m_dataMatrix;
		unsigned int m_dim;
		long int* m_offsets;

#ifdef DEBUG
		unsigned int m_numCachedQueries;
		unsigned int m_totalQueries;
#endif
};

template <class DataType, class KeyType>
CachedDistanceMatrix<DataType,KeyType>::CachedDistanceMatrix(unsigned int dim, RetrievalFunc retrievalFunc) : CachedData<DataType,KeyType>(retrievalFunc), m_dim(dim) {
	unsigned int arraySize = (dim*(dim+1))/2 - dim;
	m_dataMatrix = new DataType[arraySize];
	for (unsigned int i=0; i<arraySize; ++i) m_dataMatrix[i] = DataType(-1);
	m_offsets = new long int[dim];
	long int n = static_cast<long int>(dim);
	for (long int i=0; i<n; ++i) m_offsets[i] = (2*n*i - i*i - 3*i - 2) / 2;

#ifdef DEBUG
	m_numCachedQueries = 0;
	m_totalQueries = 0;
#endif
}

template <class DataType, class KeyType>
CachedDistanceMatrix<DataType,KeyType>::~CachedDistanceMatrix() {
	delete [] m_dataMatrix;
}

template <class DataType, class KeyType>
inline DataType* CachedDistanceMatrix<DataType,KeyType>::getDataMatrix() {
	return m_dataMatrix;
}

template <class DataType, class KeyType>
inline const DataType* CachedDistanceMatrix<DataType,KeyType>::getDataMatrix() const {
	return m_dataMatrix;
}

template <class DataType, class KeyType>
inline DataType CachedDistanceMatrix<DataType,KeyType>::get(KeyType key) {
	assert(key.first < key.second);
	unsigned int coord = m_offsets[key.first] + key.second;
	DataType value = m_dataMatrix[coord];
	if (value < DataType(0)) {
		value = this->m_rFunc(key);
		m_dataMatrix[coord] = value;
	}
#ifdef DEBUG
	else { ++m_numCachedQueries; }
	++m_totalQueries;
#endif

	return value;
}

template <class DataType, class KeyType>
inline bool CachedDistanceMatrix<DataType,KeyType>::drop(KeyType key) {
	assert(key.first < key.second);
	unsigned int coord = m_offsets[key.first] + key.second;
	if (m_dataMatrix[coord] < DataType(0)) return false;
	m_dataMatrix[coord] = DataType(-1);
	return true;
}

template <class DataType, class KeyType>
inline bool CachedDistanceMatrix<DataType,KeyType>::isCached(KeyType key) const {
	assert(key.first < key.second);
	unsigned int coord = m_offsets[key.first] + key.second;
	return (m_dataMatrix[coord] >= DataType(0));
}

} // Generic

#endif /* CACHEDDISTANCEMATRIX_H_ */
