template <class TScalar>
KmedoidMTCache<TScalar>::KmedoidMTCache(unsigned int dim) : m_dim(dim) {
	unsigned int arraySize = (dim*(dim+1))/2 - dim;
	m_data = new TScalar[arraySize];
	for (unsigned int i=0; i<arraySize; ++i) m_data[i] = TScalar(-1);
	m_offsets = new long int[dim];
	long int n = static_cast<long int>(dim);
	for (long int i=0; i<n; ++i) m_offsets[i] = (2*n*i - i*i - 3*i - 2) / 2;
}

template <class TScalar>
KmedoidMTCache<TScalar>::~KmedoidMTCache() {
	delete [] m_offsets;
	delete [] m_data;
}

template <class TScalar>
inline TScalar* KmedoidMTCache<TScalar>::operator()(unsigned int i, unsigned int j) {
	assert(i < j);
	return &(m_data[m_offsets[i] + j]);
}

template <class TScalar>
inline bool KmedoidMTCache<TScalar>::isCached(unsigned int i, unsigned int j) const {
	return m_data[m_offsets[i] + j] >= TScalar(0);
}
