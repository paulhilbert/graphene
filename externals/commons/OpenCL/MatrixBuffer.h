#ifndef OPENCLMATRIXBUFFER_H_
#define OPENCLMATRIXBUFFER_H_

#include <memory>
#include <vector>

#include <CL/cl.hpp>

#if defined(OPENCL_USE_EIGEN) || defined(OPENCL_USE_PCL)
#include <Eigen/Dense>
#ifdef OPENCL_USE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#endif
#endif

namespace OpenCL {

template <class Scalar>
class MatrixBuffer {
	public:
		typedef std::shared_ptr<MatrixBuffer> Ptr;
		typedef std::weak_ptr<MatrixBuffer> WPtr;
		typedef std::shared_ptr<cl::CommandQueue> QueuePtr;
#ifdef OPENCL_USE_EIGEN
		typedef Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> EigenMap;
#endif

	public:
		MatrixBuffer(QueuePtr queue, int rows, int cols, cl::Context& context, cl_int mode, std::shared_ptr<unsigned long> mem = nullptr);
#ifdef OPENCL_USE_EIGEN
		template <int Dim>
		MatrixBuffer(QueuePtr queue, const std::vector<Eigen::Matrix<Scalar,Dim,1>>& vectors, cl::Context& context, cl_int mode, std::shared_ptr<unsigned long> mem = nullptr);
		MatrixBuffer(QueuePtr queue, const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& matrix, cl::Context& context, cl_int mode, std::shared_ptr<unsigned long> mem = nullptr);
#endif
#ifdef OPENCL_USE_PCL
		MatrixBuffer(QueuePtr queue, const pcl::PointCloud<pcl::PointNormal>& cloud, cl::Context& context, cl_int mode, const std::vector<int>& subset = std::vector<int>(), std::shared_ptr<unsigned long> mem = nullptr);
#endif
		virtual ~MatrixBuffer();

		Scalar* data();

		cl::Buffer& buffer();
		const cl::Buffer& buffer() const;

		bool read();
		bool write() const;

#ifdef OPENCL_USE_EIGEN
		EigenMap eigenMap();
#endif
	
	protected:
		static bool checkCreateBuffer(cl_int err);
		static bool checkReadBuffer(cl_int err);
		static bool checkWriteBuffer(cl_int err);

	protected:
		QueuePtr   m_queue;
		int        m_rows;
		int        m_cols;
		Scalar*    m_data;
		cl::Buffer m_buffer;
		std::shared_ptr<unsigned long> m_mem;
};

#include "MatrixBuffer.inl"

} // OpenCL

#endif /* OPENCLMATRIXBUFFER_H_ */
