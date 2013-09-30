#ifndef SPARSEVECTOR_H_
#define SPARSEVECTOR_H_

#include <vector>
#include <set>
#include <algorithm>
using std::vector;
using std::set;



#include <Vector.h>

namespace Math {

template <class Scalar>
class SparseVector {
	public:
		SparseVector(unsigned int m_size);
		SparseVector(std::vector<Scalar> denseVec);
		virtual ~SparseVector() {}

		inline unsigned int getSize() const { return m_size; }
		std::vector<Scalar> toDense() const;

	protected:
		vector<Scalar>     m_values;
		set<unsigned int>  m_indices;
		unsigned int       m_size;
};

template <class Scalar>
SparseVector<Scalar>::SparseVector(unsigned int size) : m_size(size) {
}

template <class Scalar>
SparseVector<Scalar>::SparseVector(std::vector<Scalar> denseVec) {
	m_size = denseVec.size();
	for (unsigned int i=0; i<m_size; ++i) {
		if (fabs(denseVec[i]) < std::numeric_limits<Scalar>::epsilon()) continue;
		m_values.push_back(denseVec[i]);
		m_indices.insert(i);
	}
}

template <class Scalar>
std::vector<Scalar> SparseVector<Scalar>::toDense() const {
	std::vector<Scalar> d(m_size, Scalar(0));
	set<unsigned int>::const_iterator i = m_indices.begin();
	vector<Scalar>::const_iterator    v = m_values.begin();
	for (;i!=m_indices.end(); ++i, ++v) d[*i] = *v;
	return d;
}

} // Math

#endif /* SPARSEVECTOR_H_ */
