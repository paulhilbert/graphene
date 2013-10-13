#ifndef POINTCLOUDEDITORVIS_H_
#define POINTCLOUDEDITORVIS_H_

#include <FW/Visualizer.h>
#include <SinglePointCloud/SinglePointCloud.h>
#include <SelectModes/SelectModes.h>

#include <Vis/Colors.h>
using namespace Vis::Colors;
using Vis::RGBA;

namespace FW {


class PointCloudEditor : public SinglePointCloud, public SelectModes<Cloud, Point> {
	public:
		std::shared_ptr<PointCloudEditor> Ptr;
		std::weak_ptr<PointCloudEditor>   WPtr;
		typedef GUI::Property::Paths      Paths;

	public:
		PointCloudEditor(std::string id, const Paths& paths);
		virtual ~PointCloudEditor();

		void init();
		void render();
		void addProperties();
		void registerEvents();

		void crop();
		void erase();

	protected:
		bool resetSelectionRender();
		bool isInsideSelection(const Point& point, Input::SelectionMethod::Ptr method, Methods activeMethod);
		void updateSelectionRender(const IdxSet& selection);
};

#include "PointCloudEditor.inl"

} // FW

#endif /* POINTCLOUDEDITORVIS_H_ */
