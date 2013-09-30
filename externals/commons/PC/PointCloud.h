#ifndef POINTCLOUD_H_
#define POINTCLOUD_H_

#include <iostream>
#include <vector>
#include <memory>

#include <Eigen/Dense>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl/kdtree/kdtree_flann.h>


namespace PC {

typedef pcl::PointNormal              Point;
typedef pcl::PointCloud<Point>        Cloud;

typedef Cloud::iterator               CIter;
typedef Cloud::const_iterator         CCIter;

typedef int                           Idx;
typedef std::vector<Idx>              IdxSet;

typedef pcl::KdTreeFLANN<Point>       KdTree;

typedef Eigen::Vector3f               ScannerPosition;
typedef std::vector<ScannerPosition>  ScannerPositions;
typedef Eigen::Vector4f               Color;
typedef std::vector<Color>            Colors;

typedef enum { ABSOLUTE, RELATIVE_TO_DIAMETER, RELATIVE_TO_MINDIST } SamplingStrategy;


class PointCloud {
	public:
		typedef std::shared_ptr<PointCloud> Ptr;
		struct Impl;

	public:
		PointCloud(const PointCloud& other) = delete;
		PointCloud& operator=(const PointCloud& other) = delete;
		virtual ~PointCloud() {}

		static Ptr fromOBJFile(const fs::path& path);
		static Ptr fromPTXFile(const fs::path& path);
		static Ptr fromPTXFiles(const std::vector<fs::path>& paths);

		/// accessors ///
		Cloud::Ptr       getPointCloud() const;
		KdTree::Ptr      getKdTree() const;
		Colors           getColors() const;
		ScannerPositions getScannerPositions() const;
		IdxSet           getScanOffsets() const;

		/// observers ///
		bool hasColors() const;
		bool hasScannerPositions() const;

		float diameter() const;
		float minPointDistance() const;

		IdxSet sampleUniform(SamplingStrategy strategy, float sizeFactor) const;

	private:
		PointCloud();
		std::shared_ptr<Impl> m_impl;

/*

		static bool fromOBJ(Cloud::Ptr cloud, const fs::path& path, bool estimateNormalsIfNecessary = false);
		static bool fromScans(Cloud::Ptr cloud, const std::vector<fs::path>& paths, std::vector<Eigen::Vector3f>& scannerPositions);
		static Eigen::Vector3f fromPTX(Cloud::Ptr cloud, fs::path path);

		static Cloud::Ptr sampleUniform(Cloud::ConstPtr cloud, float cloudDiameterFactor);

		static float diameter(Cloud::ConstPtr cloud);

	protected:
		static void estimateNormals(Cloud::Ptr cloud, const Eigen::Vector3f& scannerPos);

	protected:
		PointCloud( );
		virtual ~PointCloud();
*/
};


} // PC

#endif /* POINTCLOUD_H_ */
