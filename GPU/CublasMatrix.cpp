#include "CublasMatrix.h"

namespace GPU {

CublasMatrix::CublasMatrix(int rows, int cols) : m_handle(CublasHandle::instance()), m_devPtr(NULL), m_rows(0), m_cols(0) {
	if (!rows || !cols) return;
	allocate(rows, cols);
}

CublasMatrix::~CublasMatrix() {
	discard();
}

void CublasMatrix::allocate(int rows, int cols) {
	if (rows == m_rows && cols == m_cols) return;
	discard();
	m_rows = rows;
	m_cols = cols;
	m_handle.call(cudaMalloc((void**)&m_devPtr, m_rows*m_cols*sizeof(float)));
}

void CublasMatrix::discard() {
	if (!m_devPtr) return;
	m_handle.call(cudaFree(m_devPtr));
	m_devPtr = NULL;
}

void CublasMatrix::fill(float val) {
	float* valPtr = new float[m_rows*m_cols];
	for (int i = 0; i < m_rows*m_cols; ++i) {
		valPtr[i] = val;
	}
	setFromHostPtr(valPtr, m_rows, m_cols);
	delete[] valPtr;
}

void CublasMatrix::setFromHostPtr(float* data, int rows, int cols) {
	allocate(rows, cols);
	m_handle.call(cublasSetMatrix(rows, cols, sizeof(*data), data, rows, m_devPtr, rows));
}

void CublasMatrix::setFromVectors(vector<vector<float>> data, std::function<float (float)> transform) {
	int rows = data.size();
	int cols = data[0].size();

	float* dataPtr = new float[rows*cols];
	for (int coord = 0; coord < cols; ++coord) {
		for (int point = 0; point < rows; ++point) {
			dataPtr[coord*rows+point] = transform ? transform(data[point][coord]) : data[point][coord];
		}
	}
	setFromHostPtr(dataPtr, rows, cols);
	delete[] dataPtr;
}

float* CublasMatrix::getAsHostPtr() {
	float* dataPtr = new float[m_rows*m_cols];
	m_handle.call(cublasGetMatrix(m_rows, m_cols, sizeof(float), m_devPtr, m_rows, dataPtr, m_rows));
	return dataPtr;
}

vector<vector<float>> CublasMatrix::getAsVectors() {
	float* dataPtr = getAsHostPtr();
	vector<vector<float>> data(m_rows);

	for (int point = 0; point < m_rows; ++point) {
		vector<float> part(m_cols);
		for (int coord = 0; coord < m_cols; ++coord) {
			part[coord] = dataPtr[coord*m_rows+point];
		}
		data[point] = part;
	}

	delete[] dataPtr;
	return data;
}

} // GPU
