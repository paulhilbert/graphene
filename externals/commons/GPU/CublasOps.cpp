#include "CublasOps.h"

namespace GPU {

CublasOps::CublasOps() : m_handle(CublasHandle::instance()) {

}

CublasOps::~CublasOps() {

}

void CublasOps::matMatMult(CublasMatrix& matA, CublasMatrix& matB, CublasMatrix& matResult, bool transA, bool transB, float alpha, float beta) {
	int rA = transA ? matA.numCols() : matA.numRows();
	int cA = transA ? matA.numRows() : matA.numCols();
	int cB = transB ? matB.numRows() : matB.numCols();

	m_handle.call(
		cublasSgemm(
			m_handle.getHandle(),
			transA ? CUBLAS_OP_T : CUBLAS_OP_N, transB ? CUBLAS_OP_T : CUBLAS_OP_N,
			rA, cB, cA,
			&alpha,
			matA.getDevPtr(), matA.numRows(),
			matB.getDevPtr(), matB.numRows(),
			&beta,
			matResult.getDevPtr(), rA)
	);
}

} // GPU
