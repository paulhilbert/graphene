#ifndef CUBLASHANDLE_H
#define CUBLASHANDLE_H

// STL
#include <stdexcept>

// CUDA
#include <cuda_runtime.h>
#include <cublas_v2.h>

namespace GPU {

class CublasHandle {
	public:
		static CublasHandle& instance();

		void call(cublasStatus_t status);
		void call(cudaError_t status);

		inline cublasHandle_t& getHandle() {return m_cublasHandle;}

	private:
		cublasHandle_t m_cublasHandle;

		CublasHandle();
		~CublasHandle();
		CublasHandle(const CublasHandle& other) {}
};

} // GPU

#endif
