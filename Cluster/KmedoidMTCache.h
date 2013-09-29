#ifndef KMEDOIDMTCACHE_H_
#define KMEDOIDMTCACHE_H_

#include <cassert>

namespace Cluster {

template <class TScalar>
class KmedoidMTCache {
	public:
		KmedoidMTCache(unsigned int dim);
		virtual ~KmedoidMTCache();

		TScalar* operator()(unsigned int i, unsigned int j);
		bool    isCached(unsigned int i, unsigned int j) const;

	protected:
		unsigned int   m_dim;
		TScalar*        m_data;
		long int*  m_offsets;
};

#include "KmedoidMTCache.inl"

} // Cluster

#endif /* KMEDOIDMTCACHE_H_ */
