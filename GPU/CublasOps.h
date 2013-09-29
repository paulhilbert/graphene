#ifndef CUBLASOPS_H
#define CUBLASOPS_H

#include "CublasHandle.h"
#include "CublasMatrix.h"

namespace GPU {

class CublasOps {
	public:
		CublasOps();
		~CublasOps();

		void matMatMult(CublasMatrix& matA, CublasMatrix& matB, CublasMatrix& matResult, bool transA = false, bool transB = false, float alpha = 1.f, float beta = 0.f);

	protected:
		CublasHandle& m_handle;
};

} // GPU

#endif // CUBLASOPS_H
