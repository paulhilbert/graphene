/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


inline PointCloudEditor::PointCloudEditor(std::string id, const Paths& paths, std::string upAxis, float scale, bool recenter) : Visualizer(id), SinglePointCloud(id, paths, upAxis, scale, recenter), SelectModes<Cloud,Point>(id) {
}

inline PointCloudEditor::~PointCloudEditor() {
}

inline void PointCloudEditor::init() {
	SinglePointCloud::init();
	SelectModes::init(m_cloud);
	addProperties();
	registerEvents();
}

inline void PointCloudEditor::render() {
	SinglePointCloud::render();
	SelectModes::render();
}

inline void PointCloudEditor::addProperties() {
	auto groupNormals = gui()->properties()->add<Section>("Normals", "groupNormals");
	groupNormals->collapse();
	groupNormals->add<Choice>("Method", "method")->add("knn", "kNN").add("radius", "Radius");
	groupNormals->add<Number>("Radius", "radius")->setDigits(3).setMin(0.001).setMax(30.0).setValue(0.05);
	groupNormals->add<Number>("k", "k")->setDigits(0).setMin(1).setMax(50).setValue(12);
	groupNormals->add<Bool>("Use Camera as Origin", "cam")->setValue(true);
	groupNormals->add<Button>("Compute Normals", "compute")->setCallback([&] () { computeNormals(); });
	
	auto groupEditSel = gui()->properties()->add<Group>("Edit Selection", "groupEditSel");
	auto crop = groupEditSel->add<Button>("Crop Selection", "crop");
	crop->setCallback(std::bind(&PointCloudEditor::crop, this));
	crop->disable();
	auto erase = groupEditSel->add<Button>("Erase Selection", "erase");
	erase->setCallback(std::bind(&PointCloudEditor::erase, this));
	erase->disable();
}

inline void PointCloudEditor::registerEvents() {
	fw()->events()->connect<void (FW::Events::Keys::Special)>("SPECIAL_KEY_RELEASE", [&] (FW::Events::Keys::Special key) {
		if (key == FW::Events::Keys::DEL) erase();
		if (key == FW::Events::Keys::INSERT) crop();
	});
}

inline bool PointCloudEditor::resetSelectionRender() {
	if (!m_rendered) return false;
	m_rendered->clearAnnotations();
	gui()->properties()->get<Button>({"groupEditSel", "crop"})->disable();
	gui()->properties()->get<Button>({"groupEditSel", "erase"})->disable();
	return true;
}

inline bool PointCloudEditor::isInsideSelection(const Point& point, Input::SelectionMethod::Ptr method, Methods activeMethod) {
	if (activeMethod == METHOD_AREA) {
		return std::dynamic_pointer_cast<Input::AreaSelect>(method)->pointInSelection(point.getVector3fMap());
	}
	if (activeMethod == METHOD_PAINT) {
		return std::dynamic_pointer_cast<Input::PaintSelect>(method)->pointInSelection(point.getVector3fMap());
	}
	return false;
}

inline void PointCloudEditor::updateSelectionRender(const IdxSet& selection) {
	if (!m_rendered) return;
	m_rendered->annotate(selection, "selection")->colorize(rgbaWhite());
	if (selection.size()) {
		gui()->properties()->get<Button>({"groupEditSel", "crop"})->enable();
		gui()->properties()->get<Button>({"groupEditSel", "erase"})->enable();
	} else {
		gui()->properties()->get<Button>({"groupEditSel", "crop"})->disable();
		gui()->properties()->get<Button>({"groupEditSel", "erase"})->disable();
	}
}

inline void PointCloudEditor::computeNormals() {
	bool knn = gui()->properties()->get<Choice>({"groupNormals", "method"})->value() == "knn";
	bool cam = gui()->properties()->get<Bool>({"groupNormals", "cam"})->value();

	pcl::NormalEstimation<Point, Point> ne;
	if (knn) {
		ne.setKSearch(gui()->properties()->get<Number>({"groupNormals", "k"})->value());
	} else {
		ne.setRadiusSearch(gui()->properties()->get<Number>({"groupNormals", "radius"})->value());
	}
	Eigen::Vector3f viewPos = cam ? fw()->transforms()->cameraPosition() : m_cloud->sensor_origin_.head(3);
	ne.setViewPoint(0.f, 0.f, 0.f);
	for (auto& p : *m_cloud) {
		p.getVector3fMap() -= viewPos;
	}
	ne.setInputCloud(m_cloud);
	ne.compute(*m_cloud);
	for (auto& p : *m_cloud) {
		p.getVector3fMap() += viewPos;
	}
	gui()->log()->info("Computed normals.");
}

inline void PointCloudEditor::crop() {
	if (!m_selection.size()) return;
	*m_cloud = Cloud(*m_cloud, m_selection);
	resetSelection();
	m_rendered->setFromPCLCloud(m_cloud->begin(), m_cloud->end());
}

inline void PointCloudEditor::erase() {
	if (!m_selection.size()) return;
	IdxSet invert(m_cloud->size());
	std::iota(invert.begin(), invert.end(), 0);
	Algorithm::remove(invert, [&] (int idx) { return std::binary_search(m_selection.begin(), m_selection.end(), idx); });
	*m_cloud = Cloud(*m_cloud, invert);
	resetSelection();
	m_rendered->setFromPCLCloud(m_cloud->begin(), m_cloud->end());
}
