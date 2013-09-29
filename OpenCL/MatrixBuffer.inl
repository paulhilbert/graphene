template <class Scalar>
MatrixBuffer<Scalar>::MatrixBuffer(QueuePtr queue, int rows, int cols, cl::Context& context, cl_int mode, std::shared_ptr<unsigned long> mem) : m_queue(queue), m_rows(rows), m_cols(cols), m_data(NULL), m_mem(mem) {
	m_data = new Scalar[m_rows*m_cols];

	cl_int err;
	m_buffer = cl::Buffer(context, mode, sizeof(Scalar)*m_rows*m_cols, NULL, &err);
	if (m_mem) *m_mem += sizeof(Scalar)*m_rows*m_cols;
	if (!checkCreateBuffer(err)) return;
}

#ifdef OPENCL_USE_EIGEN
template <class Scalar>
template <int Dim>
MatrixBuffer<Scalar>::MatrixBuffer(QueuePtr queue, const std::vector<Eigen::Matrix<Scalar,Dim,1>>& vectors, cl::Context& context, cl_int mode, std::shared_ptr<unsigned long> mem) : m_queue(queue), m_mem(mem) {
	m_rows = vectors.size();
	m_cols = Dim;
	m_data = new Scalar[m_rows*m_cols];
	for (unsigned int i=0; i<m_rows; ++i) {
		for (unsigned int j=0; j<m_cols; ++j) {
			m_data[i*m_cols+j] = vectors[i][j];
		}
	}

	cl_int err;
	m_buffer = cl::Buffer(context, mode, sizeof(Scalar)*m_rows*m_cols, NULL, &err);
	if (m_mem) *m_mem += sizeof(Scalar)*m_rows*m_cols;
	if (!checkCreateBuffer(err)) return;
}

template <class Scalar>
MatrixBuffer<Scalar>::MatrixBuffer(QueuePtr queue, const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& matrix, cl::Context& context, cl_int mode, std::shared_ptr<unsigned long> mem) : m_queue(queue), m_mem(mem) {
	m_rows = matrix.rows();
	m_cols = matrix.cols();
	m_data = new Scalar[m_rows*m_cols];
	for (unsigned int i=0; i<m_rows; ++i) {
		for (unsigned int j=0; j<m_cols; ++j) {
			m_data[i*m_cols+j] = matrix(i,j);
		}
	}

	cl_int err;
	m_buffer = cl::Buffer(context, mode, sizeof(Scalar)*m_rows*m_cols, NULL, &err);
	if (m_mem) *m_mem += sizeof(Scalar)*m_rows*m_cols;
	if (!checkCreateBuffer(err)) return;
}
#endif

#ifdef OPENCL_USE_PCL
template <class Scalar>
MatrixBuffer<Scalar>::MatrixBuffer(QueuePtr queue, const pcl::PointCloud<pcl::PointNormal>& cloud, cl::Context& context, cl_int mode, const std::vector<int>& subset, std::shared_ptr<unsigned long> mem) : m_queue(queue), m_mem(mem) {
	std::vector<int> indices(subset);
	if (!indices.size()) {
		indices.resize(cloud.size());
		std::iota(indices.begin(), indices.end(), 0);
	}
	m_rows = indices.size();
	m_cols = 6;
	m_data = new Scalar[m_rows*m_cols];
	for (unsigned int i=0; i<m_rows; ++i) {
		m_data[i*m_cols+0] = cloud.points[indices[i]].x;
		m_data[i*m_cols+1] = cloud.points[indices[i]].y;
		m_data[i*m_cols+2] = cloud.points[indices[i]].z;
		m_data[i*m_cols+3] = cloud.points[indices[i]].normal[0];
		m_data[i*m_cols+4] = cloud.points[indices[i]].normal[1];
		m_data[i*m_cols+5] = cloud.points[indices[i]].normal[2];
	}

	cl_int err;
	m_buffer = cl::Buffer(context, mode, sizeof(Scalar)*m_rows*m_cols, NULL, &err);
	if (m_mem) *m_mem += sizeof(Scalar)*m_rows*m_cols;
	if (!checkCreateBuffer(err)) return;
}
#endif

template <class Scalar>
MatrixBuffer<Scalar>::~MatrixBuffer() {
	if (m_mem) *m_mem -= sizeof(Scalar)*m_rows*m_cols;
	delete [] m_data;
}

template <class Scalar>
Scalar* MatrixBuffer<Scalar>::data() {
	return m_data;
}

template <class Scalar>
cl::Buffer& MatrixBuffer<Scalar>::buffer() {
	return m_buffer;
}

template <class Scalar>
const cl::Buffer& MatrixBuffer<Scalar>::buffer() const {
	return m_buffer;
}

template <class Scalar>
inline bool MatrixBuffer<Scalar>::read() {
	cl_int err = m_queue->enqueueReadBuffer(m_buffer, CL_TRUE, 0, sizeof(Scalar)*m_rows*m_cols, m_data);
	return checkReadBuffer(err);
}

template <class Scalar>
inline bool MatrixBuffer<Scalar>::write() const {
	cl_int err = m_queue->enqueueWriteBuffer(m_buffer, CL_TRUE, 0, sizeof(Scalar)*m_rows*m_cols, m_data);
	return checkWriteBuffer(err);
}

#ifdef OPENCL_USE_EIGEN
template <class Scalar>
typename MatrixBuffer<Scalar>::EigenMap MatrixBuffer<Scalar>::eigenMap() {
	return EigenMap(m_data, m_rows, m_cols);
}
#endif

template <class Scalar>
inline bool MatrixBuffer<Scalar>::checkCreateBuffer(cl_int err) {
	if (err != CL_SUCCESS) {
		Log::error("Unable to create buffer object. Reason:");
		switch (err) {
			case CL_INVALID_CONTEXT: Log::error("CL_INVALID_CONTEXT"); break;
			case CL_INVALID_VALUE: Log::error("CL_INVALID_VALUE"); break;
			case CL_INVALID_BUFFER_SIZE: Log::error("CL_INVALID_BUFFER_SIZE"); break;
			case CL_INVALID_HOST_PTR: Log::error("CL_INVALID_HOST_PTR"); break;
			case CL_MEM_OBJECT_ALLOCATION_FAILURE: Log::error("CL_MEM_OBJECT_ALLOCATION_FAILURE"); break;
			case CL_OUT_OF_RESOURCES: Log::error("CL_OUT_OF_RESOURCES"); break;
			case CL_OUT_OF_HOST_MEMORY: Log::error("CL_OUT_OF_HOST_MEMORY"); break;
			default: Log::error("Unknown reason");
		}
		return false;
	}
	return true;
}

template <class Scalar>
inline bool MatrixBuffer<Scalar>::checkReadBuffer(cl_int err) {
	if (err != CL_SUCCESS) {
		Log::error("Unable to read buffer. Reason:");
		switch (err) {
			case CL_INVALID_CONTEXT: Log::error("CL_INVALID_CONTEXT"); break;
			case CL_INVALID_MEM_OBJECT: Log::error("CL_INVALID_MEM_OBJECT"); break;
			case CL_INVALID_VALUE: Log::error("CL_INVALID_VALUE"); break;
			case CL_INVALID_EVENT_WAIT_LIST: Log::error("CL_INVALID_EVENT_WAIT_LIST"); break;
			case CL_MISALIGNED_SUB_BUFFER_OFFSET: Log::error("CL_MISALIGNED_SUB_BUFFER_OFFSET"); break;
			case CL_MEM_OBJECT_ALLOCATION_FAILURE: Log::error("CL_MEM_OBJECT_ALLOCATION_FAILURE"); break;
			case CL_OUT_OF_RESOURCES: Log::error("CL_OUT_OF_RESOURCES"); break;
			case CL_OUT_OF_HOST_MEMORY: Log::error("CL_OUT_OF_HOST_MEMORY"); break;
			default: Log::error("Unknown reason..."); break;
		}
		return false;
	}
	return true;
}

template <class Scalar>
inline bool MatrixBuffer<Scalar>::checkWriteBuffer(cl_int err) {
	if (err != CL_SUCCESS) {
		Log::error("Unable to write buffer. Reason:");
		switch (err) {
			case CL_INVALID_CONTEXT: Log::error("CL_INVALID_CONTEXT"); break;
			case CL_INVALID_MEM_OBJECT: Log::error("CL_INVALID_MEM_OBJECT"); break;
			case CL_INVALID_VALUE: Log::error("CL_INVALID_VALUE"); break;
			case CL_INVALID_EVENT_WAIT_LIST: Log::error("CL_INVALID_EVENT_WAIT_LIST"); break;
			case CL_MISALIGNED_SUB_BUFFER_OFFSET: Log::error("CL_MISALIGNED_SUB_BUFFER_OFFSET"); break;
			case CL_MEM_OBJECT_ALLOCATION_FAILURE: Log::error("CL_MEM_OBJECT_ALLOCATION_FAILURE"); break;
			case CL_OUT_OF_RESOURCES: Log::error("CL_OUT_OF_RESOURCES"); break;
			case CL_OUT_OF_HOST_MEMORY: Log::error("CL_OUT_OF_HOST_MEMORY"); break;
			default: Log::error("Unknown reason..."); break;
		}
		return false;
	}
	return true;
}
