#include "PointCloud.h"

#include <fstream>
#include <vector>
#include <algorithm>

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <IO/Log.h>
using namespace IO;

namespace PC {


struct PointCloud::Impl {
	Impl();

	static std::shared_ptr<Impl> fromOBJFile(const fs::path& path);
	static std::shared_ptr<Impl> fromPTXFile(const fs::path& path);
	static std::shared_ptr<Impl> fromPTXFiles(const std::vector<fs::path>& paths);

	Cloud::Ptr getPointCloud() const;
	KdTree::Ptr getKdTree() const;

	Colors getColors() const;
	ScannerPositions getScannerPositions() const;
	IdxSet getScanOffsets() const;

	bool hasColors() const;
	bool hasScannerPositions() const;

	float diameter() const;
	float minPointDistance() const;

	void loadOBJFile(const fs::path& path);
	void loadPTXFile(const fs::path& path, bool updateKdTree = true);
	void loadPTXFiles(const std::vector<fs::path>& paths);

	IdxSet sampleUniform(SamplingStrategy strategy, float sizeFactor) const;

	void estimateNormals(Cloud::Ptr cloud, const ScannerPosition& spos) const;

	Cloud::Ptr m_cloud;
	KdTree::Ptr m_kdtree;
	bool m_hasColors;
	bool m_hasScannerPositions;
	Colors m_colors;
	ScannerPositions m_scannerPos;
	IdxSet m_scanOffsets;
	bool m_loaded;
};

/// POINTCLOUD ///

PointCloud::Ptr PointCloud::fromOBJFile(const fs::path& path) {
	Ptr self(new PointCloud());
	self->m_impl = Impl::fromOBJFile(path);
	return self;
}

PointCloud::Ptr PointCloud::fromPTXFile(const fs::path& path) {
	Ptr self(new PointCloud());
	self->m_impl = Impl::fromPTXFile(path);
	return self;
}

PointCloud::Ptr PointCloud::fromPTXFiles(const std::vector<fs::path>& paths) {
	Ptr self(new PointCloud());
	self->m_impl = Impl::fromPTXFiles(paths);
	return self;
}

Cloud::Ptr PointCloud::getPointCloud() const {
	return m_impl->getPointCloud();
}

KdTree::Ptr PointCloud::getKdTree() const {
	return m_impl->getKdTree();
}

Colors PointCloud::getColors() const {
	return m_impl->getColors();
}

ScannerPositions PointCloud::getScannerPositions() const {
	return m_impl->getScannerPositions();
}

IdxSet PointCloud::getScanOffsets() const {
	return m_impl->getScanOffsets();
}

bool PointCloud::hasColors() const {
	return m_impl->hasColors();
}

bool PointCloud::hasScannerPositions() const {
	return m_impl->hasScannerPositions();
}

float PointCloud::diameter() const {
	return m_impl->diameter();
}

float PointCloud::minPointDistance() const {
	return m_impl->minPointDistance();
}

IdxSet PointCloud::sampleUniform(SamplingStrategy strategy, float sizeFactor) const {
	return m_impl->sampleUniform(strategy, sizeFactor);
}

PointCloud::PointCloud() : m_impl(new Impl()) {
}

/// POINTCLOUD::IMPL ///

PointCloud::Impl::Impl() : m_cloud(new Cloud()), m_kdtree(new KdTree()), m_hasColors(false), m_hasScannerPositions(false), m_loaded(false) {
}

std::shared_ptr<PointCloud::Impl> PointCloud::Impl::fromOBJFile(const fs::path& path) {
	std::shared_ptr<Impl> self(new Impl());
	self->loadOBJFile(path);
	return self;
}

std::shared_ptr<PointCloud::Impl> PointCloud::Impl::fromPTXFile(const fs::path& path) {
	std::shared_ptr<Impl> self(new Impl());
	self->loadPTXFile(path);
	return self;
}

std::shared_ptr<PointCloud::Impl> PointCloud::Impl::fromPTXFiles(const std::vector<fs::path>& paths) {
	std::shared_ptr<Impl> self(new Impl());
	self->loadPTXFiles(paths);
	return self;
}

Cloud::Ptr PointCloud::Impl::getPointCloud() const {
	return m_cloud;
}

KdTree::Ptr PointCloud::Impl::getKdTree() const {
	return m_kdtree;
}

Colors PointCloud::Impl::getColors() const {
	return m_colors;
}

ScannerPositions PointCloud::Impl::getScannerPositions() const {
	return m_scannerPos;
}

IdxSet PointCloud::Impl::getScanOffsets() const {
	return m_scanOffsets;
}

bool PointCloud::Impl::hasColors() const {
	return m_hasColors;
}

bool PointCloud::Impl::hasScannerPositions() const {
	return m_hasScannerPositions;
}

float PointCloud::Impl::diameter() const {
	if (!m_loaded) return 0.f;
	Eigen::AlignedBox<float,3> bb;
	for (const auto& p : *m_cloud) bb.extend(p.getVector3fMap());
	return bb.diagonal().norm();
}

float PointCloud::Impl::minPointDistance() const {
	if (!m_loaded) return 0.f;
	IdxSet closest(2);
	std::vector<float> dists(2);
	float result = std::numeric_limits<float>::max();
	for (const auto& p : *m_cloud) {
		m_kdtree->nearestKSearch(p, 2, closest, dists);
		result = std::min(result, dists[1]);
	}
	return result;
}

void PointCloud::Impl::loadOBJFile(const fs::path& path) {
	if (!fs::exists(path)) {
		Log::error("Path \""+path.string()+"\" to pointcloud does not exist.");
		return;
	}
	std::ifstream in(path.string().c_str());
	if (!in.good()) {
		Log::error("Bad input stream");
		return;
	}

	Log::info("Loading point cloud...", Console::FLUSH);
	std::string line;
	std::vector<std::tuple<float,float,float>> points, normals;
	char type[3];
	float v0, v1, v2;
	std::string typeStr;
	while (std::getline(in, line)) {
		if (sscanf(line.c_str(), "%2s %f %f %f", type, &v0, &v1, &v2) == 4) {
			typeStr = type;
			if (typeStr == "v") {
				points.push_back(std::make_tuple(v0,v1,v2));
			} else if (typeStr == "vn") {
				normals.push_back(std::make_tuple(v0,v1,v2));
			}
		}
	}
	in.close();

	bool noNormals = false;
	if (!points.size()) {
		Log::error("No vertices in point cloud");
		return;
	}
	if (!normals.size()) { Log::info("No normals in point cloud file."); noNormals = true; }
	else if (normals.size() != points.size()) { Log::info("Inconsistent normal and vertex count. Treating normals as non-existent."); noNormals = true; }

	m_cloud->clear();
	Point p;
	for (unsigned int i=0; i<points.size(); ++i) {
		std::tie(p.x, p.y, p.z) = points[i];
		if (!noNormals) std::tie(p.normal[0], p.normal[1], p.normal[2]) = normals[i];
		m_cloud->push_back(p);
	}

	Log::info("Loaded point cloud.", Console::FLUSH | Console::CLEAR | Console::END);
	Log::info("Point count: "+lexical_cast<std::string>(m_cloud->size())+".");
	//if (noNormals && estimateNormalsIfNecessary) estimateNormals(cloud);
	m_kdtree->setInputCloud(m_cloud);
	m_loaded = true;
}

void PointCloud::Impl::loadPTXFile(const fs::path& path, bool updateKdTree) {
	if (!fs::exists(path)) {
		Log::error("Path \""+path.string()+"\" to pointcloud does not exist.");
		return;
	}
	if (path.extension() != ".ptx") return;
	
	std::ifstream in(path.string().c_str());
	if (!in.good()) {
		Log::error("Bad input stream");
		return;
	}
	Log::info("Loading point cloud...", Console::FLUSH);
	std::string line;

	// count
	int w,h;
	std::getline(in,line);
	sscanf(line.c_str(), "%d", &w);
	std::getline(in,line);
	sscanf(line.c_str(), "%d", &h);
	//int count = w*h;

	// scanner pos
	Eigen::Vector3f spos;
	std::getline(in,line);
	sscanf(line.c_str(), "%f %f %f", &(spos.data()[0]), &(spos.data()[1]), &(spos.data()[2]));
	// dirty skip
	std::getline(in,line);
	std::getline(in,line);
	std::getline(in,line);

	// transformation
	Eigen::Matrix4f trans;
	std::getline(in,line);
	sscanf(line.c_str(), "%f %f %f %f", &(trans.data()[ 0]), &(trans.data()[ 1]), &(trans.data()[ 2]), &(trans.data()[ 3]));
	std::getline(in,line);
	sscanf(line.c_str(), "%f %f %f %f", &(trans.data()[ 4]), &(trans.data()[ 5]), &(trans.data()[ 6]), &(trans.data()[ 7]));
	std::getline(in,line);
	sscanf(line.c_str(), "%f %f %f %f", &(trans.data()[ 8]), &(trans.data()[ 9]), &(trans.data()[10]), &(trans.data()[11]));
	std::getline(in,line);
	sscanf(line.c_str(), "%f %f %f %f", &(trans.data()[12]), &(trans.data()[13]), &(trans.data()[14]), &(trans.data()[15]));

	Cloud::Ptr cloud(new Cloud());
	Point p;
	while (std::getline(in, line)) {
		if (sscanf(line.c_str(), "%f %f %f%*[^\n]", &p.x, &p.y, &p.z) == 3) {
			cloud->push_back(p);
		}
	}
	in.close();
	pcl::transformPointCloud(*cloud, *cloud, trans);

	Log::info("Loaded point cloud.", Console::FLUSH | Console::CLEAR | Console::END);
	Log::info("Added point count: "+lexical_cast<std::string>(cloud->size())+".");
	//estimateNormals(cloud, spos);
	m_scanOffsets.push_back(m_cloud->size());
	m_cloud->insert(m_cloud->end(), cloud->begin(), cloud->end());
	if (updateKdTree) m_kdtree->setInputCloud(m_cloud);
	m_scannerPos.push_back(spos);
	m_hasScannerPositions = true;
	m_loaded = true;
}

void PointCloud::Impl::loadPTXFiles(const std::vector<fs::path>& paths) {
	for (auto& p : paths) loadPTXFile(p, false);
	m_kdtree->setInputCloud(m_cloud);
}

IdxSet PointCloud::Impl::sampleUniform(SamplingStrategy strategy, float sizeFactor) const {
	float size = sizeFactor;
	switch (strategy) {
		case RELATIVE_TO_DIAMETER: size *= diameter(); break;
		case RELATIVE_TO_MINDIST:  size *= minPointDistance(); break;
		default: break; // else consider sizeFactor to be absolute
	}
	pcl::UniformSampling<Point> us;
	us.setInputCloud(m_cloud);
	us.setRadiusSearch(size);
	pcl::PointCloud<int> subset;
	us.compute(subset);
	std::sort(subset.points.begin (), subset.points.end ());
	IdxSet result(subset.size());
	std::transform(subset.begin(), subset.end(), result.begin(), [&] (int idx) { return idx; });
	return result;
}

void PointCloud::Impl::estimateNormals(Cloud::Ptr cloud, const ScannerPosition& spos) const {
	pcl::NormalEstimation<Point, Point> ne;
	ne.setInputCloud(cloud);

	pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch (0.03);
	ne.setViewPoint(spos[0], spos[1], spos[2]);

	ne.compute(*cloud);
}

} // PC
