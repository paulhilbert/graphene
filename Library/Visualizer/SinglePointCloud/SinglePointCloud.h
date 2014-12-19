/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef SINGLEPOINTCLOUDVIS_H_
#define SINGLEPOINTCLOUDVIS_H_


#include <pcl/keypoints/uniform_sampling.h>

#include <Library/Geometry/PCLTools.h>
using Geometry::PCLTools;
typedef pcl::PointXYZRGBNormal  Point;
typedef PCLTools<Point>         Tools;
typedef Tools::CloudType        Cloud;
typedef Tools::Idx              Idx;
typedef Tools::IdxSet           IdxSet;

#include <FW/FWVisualizer.h>

#include <harmont/pcl_traits.hpp>

namespace FW {


class SinglePointCloud : virtual public  Visualizer {
	public:
		typedef pcl::PointXYZRGBNormal              PointT;
		typedef harmont::cloud<PointT>              CloudT;
		typedef harmont::pointcloud_object<CloudT, boost::shared_ptr>  RenderedCloudT;

	public:
		SinglePointCloud(std::string id, const GUI::Property::Paths& paths, std::string upAxis, float scale, bool recenter);
		virtual ~SinglePointCloud();

		void init();
		void addProperties();
        void addModes();
		void registerEvents();

	protected:
		void addClouds(const GUI::Property::Paths& paths);
		void exportCloud(const fs::path& path);

	protected:
		GUI::Property::Paths         m_paths;
        RenderedCloudT::ptr_t        m_cloud;
		std::string                  m_upAxis;
		float                        m_scale;
		bool                         m_recenter;
};

#include "SinglePointCloud.inl"

} // FW

#endif /* SINGLEPOINTCLOUDVIS_H_ */
