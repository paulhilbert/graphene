#include "CublasHandle.h"

namespace GPU {

CublasHandle& CublasHandle::instance() {
	static CublasHandle instance;
	return instance;
}

void CublasHandle::call(cublasStatus_t status) {
	if (status != CUBLAS_STATUS_SUCCESS) {
		throw std::runtime_error("CUBLAS error");
	}
}

void CublasHandle::call(cudaError_t status) {
	if (status != cudaSuccess) {
		throw std::runtime_error("CUDA error");
	}
}

CublasHandle::CublasHandle() {
	call(cublasCreate(&m_cublasHandle));
}

CublasHandle::~CublasHandle() {
	call(cublasDestroy(m_cublasHandle));
}

} // GPU
