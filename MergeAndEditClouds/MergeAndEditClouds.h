#ifndef MERGEANDEDITCLOUDSVIS_H_
#define MERGEANDEDITCLOUDSVIS_H_

#include <FW/Visualizer.h>
#include <FW/Factory.h>

#include <Geometry/PCLTools.h>
using Geometry::PCLTools;
typedef pcl::PointNormal Point;
typedef PCLTools<Point>  Tools;
typedef Tools::CloudType Cloud;
typedef Tools::Idx       Idx;
typedef Tools::IdxSet    IdxSet;

#include <Library/Rendered/Cloud.h>
#include <Library/Input/AreaSelect.h>
#include <Library/Input/PaintSelect.h>


namespace FW {


class MergeAndEditClouds : public Visualizer {
	public:
		class Factory;

	public:
		MergeAndEditClouds(std::string id, const GUI::Property::Paths& paths);
		virtual ~MergeAndEditClouds();

		void init();
		void render();
		void addProperties();
		void addModes();
		void registerEvents();

	protected:
		void addClouds(const GUI::Property::Paths& paths);
		void exportCloud(const fs::path& path);
		void uploadCloud();
		void setupSelection();
		void resetSelection();
		void crop();
		void erase();
		void resample();

	protected:
		GUI::Property::Paths     m_paths;
		Cloud::Ptr               m_cloud;
		Rendered::Cloud::Ptr     m_rendered;

		IdxSet                   m_selection;
		Input::AreaSelect::Ptr   m_areaSelect;
		Input::PaintSelect::Ptr  m_paintSelect;
};


class MergeAndEditClouds::Factory : public FW::Factory {
	public:
		Factory();
		virtual ~Factory();

		void init();
		Visualizer::Ptr addVisualizer();
};

} // FW

#endif /* MERGEANDEDITCLOUDSVIS_H_ */
