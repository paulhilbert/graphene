/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef POINTCLOUDEDITORVIS_H_
#define POINTCLOUDEDITORVIS_H_

#include <include/common.h>
#include <FW/FWVisualizer.h>
#include <Library/Visualizer/SinglePointCloud/SinglePointCloud.h>
#include <Library/Visualizer/SelectModes/SelectModes.h>


namespace FW {

class PointCloudEditor : public SinglePointCloud, public SelectModes<Cloud, Point> {
	public:
		std::shared_ptr<PointCloudEditor> Ptr;
		std::weak_ptr<PointCloudEditor>   WPtr;
		typedef GUI::Property::Paths      Paths;

	public:
		PointCloudEditor(std::string id, const Paths& paths, std::string upAxis, float scale, bool recenter);
		virtual ~PointCloudEditor();

		void init();
		void render(ShaderProgram& program);
		void addProperties();
		void registerEvents();

		void computeNormals();
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
