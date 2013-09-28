#ifndef DATABUFFER_H_
#define DATABUFFER_H_

#include <iostream>
#include <cstring>

#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES
#endif
#include <GL/glew.h>
#include <vector>

namespace Buffer {

typedef enum {STATIC_ACCESS = GL_STATIC_DRAW, DYNAMIC_ACCESS = GL_DYNAMIC_DRAW} AccessMethod;

template <class Type>
class DataBuffer {
	public:
		DataBuffer() : m_data(NULL), m_id(0), m_dataSize(0) {
		}
		virtual ~DataBuffer() {
			if (m_id) glDeleteBuffers(1, &m_id);
			delete [] m_data;
			m_data = NULL;
			m_id = 0;
		}

		void init() {
			glGenBuffers(1, &m_id);
			setTypeId(Type(0));
		}

		virtual void set(std::vector<Type> data) {
			if (!m_data) allocate(data.size());
			for (unsigned int i=0; i < data.size(); ++i) { m_data[i] = data[i]; }
		}

		virtual void upload(AccessMethod aMethod) = 0;

		virtual void bind() = 0;
		virtual void release() = 0;

		inline GLuint getId() const {return m_id;}
		inline unsigned int getDataSize() const {return m_dataSize;}

	protected:
		inline void allocate(unsigned int numValues) {
			m_data = new Type[numValues];
			m_dataSize = numValues;
		}

		inline unsigned int allocateAdditional(unsigned int numValues) {
			Type* newArray = new Type[numValues+m_dataSize];
			memcpy((void*)newArray, (const void*)m_data, m_dataSize*sizeof(Type));
			delete [] m_data;
			m_data = newArray;
			unsigned int startIndex = m_dataSize;
			m_dataSize += numValues;
			return startIndex;
		}

		void uploadData(int target, AccessMethod aMethod) {
			glBindBuffer(target, m_id);
			glBufferData(target, sizeof(Type)*m_dataSize, m_data, aMethod);
		}

		void setTypeId(float) {
			m_typeId = GL_FLOAT;
		}
		void setTypeId(double) {
			m_typeId = GL_DOUBLE;
		}
		void setTypeId(int) {
			m_typeId = GL_INT;
		}
		void setTypeId(unsigned int) {
			m_typeId = GL_UNSIGNED_INT;
		}


		Type*   m_data;
		GLuint  m_id;
		GLuint  m_typeId;
		unsigned int m_dataSize;
};


} // Buffer

#endif /* DATABUFFER_H_ */
