#ifndef CUBLASMATRIX_H
#define CUBLASMATRIX_H

#include <functional>
#include <vector>
using std::vector;

#include "CublasHandle.h"

namespace GPU {

class CublasMatrix {
	public:
		CublasMatrix(int rows = 0, int cols = 0);
		virtual ~CublasMatrix();

		void allocate(int rows, int cols);
		void discard();

		inline int numRows() {return m_rows;}
		inline int numCols() {return m_cols;}

		void fill(float val);
		void setFromHostPtr(float* data, int rows, int cols);
		void setFromVectors(vector<vector<float>> data, std::function<float (float)> transform = NULL);

		inline float* getDevPtr() {return m_devPtr;}
		float* getAsHostPtr();
		vector<vector<float>> getAsVectors();

	protected:
		CublasHandle& m_handle;

		float*   m_devPtr;
		int      m_rows;
		int      m_cols;
};

} // GPU

#endif // CUBLASMATRIX_H
