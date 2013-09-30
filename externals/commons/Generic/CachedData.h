#ifndef CACHEDDATA_H_
#define CACHEDDATA_H_

#include <map>
using std::map;

namespace Generic {

template <class DataType, class KeyType>
class CachedData {
	public:
		typedef boost::function<DataType (KeyType)> RetrievalFunc;
		typedef map<KeyType, DataType> Data;
		typedef typename Data::iterator DataIter;
		typedef typename Data::const_iterator DataConstIter;

	public:
		CachedData(RetrievalFunc retrievalFunc);
		virtual ~CachedData() {}

		Data& getData();
		const Data& getData() const;

		virtual DataType get(KeyType key);
		bool drop(KeyType key);
		bool isCached(KeyType key) const;
		DataIter getDataIterator(KeyType key);
		DataConstIter getDataIterator(KeyType key) const;

	protected:
		RetrievalFunc m_rFunc;
		Data m_data;
};

template <class DataType, class KeyType>
CachedData<DataType,KeyType>::CachedData(RetrievalFunc retrievalFunc) : m_rFunc(retrievalFunc) {
}

template <class DataType, class KeyType>
inline typename CachedData<DataType,KeyType>::Data& CachedData<DataType,KeyType>::getData() {
	return m_data;
}

template <class DataType, class KeyType>
inline const typename CachedData<DataType,KeyType>::Data& CachedData<DataType,KeyType>::getData() const {
	return m_data;
}

template <class DataType, class KeyType>
inline DataType CachedData<DataType,KeyType>::get(KeyType key) {
	DataIter it = getDataIterator(key);
	if (it != m_data.end()) return it->second;
	DataType value = m_rFunc(key);
	m_data[key] = value;
	return value;
}

template <class DataType, class KeyType>
inline bool CachedData<DataType,KeyType>::drop(KeyType key) {
	DataIter it = getDataIterator(key);
	if (it != m_data.end) {
		m_data.erase(it);
		return true;
	}
	return false;
}

template <class DataType, class KeyType>
inline bool CachedData<DataType,KeyType>::isCached(KeyType key) const {
	return m_data.find(key) != m_data.end();
}

template <class DataType, class KeyType>
inline typename CachedData<DataType,KeyType>::DataIter CachedData<DataType,KeyType>::getDataIterator(KeyType key) {
	return m_data.find(key);
}

template <class DataType, class KeyType>
inline typename CachedData<DataType,KeyType>::DataConstIter CachedData<DataType,KeyType>::getDataIterator(KeyType key) const {
	return m_data.find(key);
}

} // Generic

#endif /* CACHEDDATA_H_ */
