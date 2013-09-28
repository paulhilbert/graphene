#ifndef VBO_H_
#define VBO_H_

#include <Eigen/Dense>

#include "DataBuffer.h"

namespace Buffer {

template <class Type, int VecDim>
class VBO : public DataBuffer<Type> {
	public:
		typedef Eigen::Matrix<Type,VecDim,1> Vec;
	public:
		VBO() : DataBuffer<Type>() {}
		virtual ~VBO() {}

		void set(const std::vector<Vec>& data) {
			unsigned int dataSize = VecDim * data.size();
			if (dataSize != this->m_dataSize) this->allocate(dataSize);
			for (unsigned int i=0; i < data.size(); ++i) {
				for (unsigned int d=0; d < VecDim; ++d) {
					this->m_data[i*VecDim + d] = data[i][d];
				}
			}
		}

		void add(const std::vector<Vec>& data) {
			unsigned int dataSize = VecDim * data.size();
			unsigned int startIndex = this->allocateAdditional(dataSize);
			for (unsigned int i=0; i < data.size(); ++i) {
				for (unsigned int d=0; d < VecDim; ++d) {
					this->m_data[startIndex + i*VecDim + d] = data[i][d];
				}
			}
		}

		void upload(AccessMethod aMethod) {
			this->uploadData(GL_ARRAY_BUFFER, aMethod);
		}

		void bind() {
			glBindBuffer(GL_ARRAY_BUFFER, this->m_id);
		}

		void release() {
			glBindBuffer(GL_ARRAY_BUFFER, 0);
		}
};


} // Buffer

#endif /* VBO_H_ */
