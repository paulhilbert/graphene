/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef MULTIPOINTCLOUDVIS_H_
#define MULTIPOINTCLOUDVIS_H_

#include <FW/FWVisualizer.h>

#include <Geometry/PCLTools.h>
using Geometry::PCLTools;
typedef pcl::PointNormal Point;
typedef PCLTools<Point>  Tools;
typedef Tools::CloudType Cloud;
typedef Tools::Idx       Idx;
typedef Tools::IdxSet    IdxSet;

#include <Library/Rendered/Cloud.h>
#include <Library/Rendered/Vectors.h>
#include <Library/Rendered/Lines.h>

namespace FW {


class MultiPointCloud : virtual public Visualizer {
	public:
		typedef std::shared_ptr<MultiPointCloud> Ptr;
		typedef std::weak_ptr<MultiPointCloud>   WPtr;
		typedef Rendered::Field                  RF;
		typedef Rendered::Cloud                  RC;
		typedef Rendered::Vectors                RV;
		typedef Rendered::Lines                  RL;
		typedef std::map<std::string, RF::Ptr>   RFPtrs;
		friend class Computation;

	public:
		MultiPointCloud(std::string id, const GUI::Property::Paths& paths);
		virtual ~MultiPointCloud();

		void init();
		void render(ShaderProgram& program);
		void addProperties();
		void registerEvents();

		virtual BoundingBox boundingBox() const;

	protected:
		void    addClouds(const GUI::Property::Paths& paths);
		RC::Ptr addCloud(std::string name, RGBA color, Cloud::Ptr cloud, bool visible = true, bool ignoreNormals = false);
		RC::Ptr addCloud(std::string name, Cloud::Ptr cloud, std::vector<RGBA>* color, bool visible = true, bool ignoreNormals = false);
		RV::Ptr addNormals(std::string name, RGBA color, Cloud::Ptr cloud, bool visible = true, float factor = 1.f);
		RL::Ptr addLines(std::string name, RGBA color, const std::vector<Vector3f>& points, bool visible = true);
		void    removeField(std::string name);
		void    exportCloud(const fs::path& path);
		void    resample();

	protected:
		GUI::Property::Paths     m_paths;
		Cloud::Ptr               m_cloud;
		RFPtrs                   m_rf;
		BoundingBox              m_bbox;
};

#include "MultiPointCloud.inl"

} // FW

#endif /* MULTIPOINTCLOUDVIS_H_ */
