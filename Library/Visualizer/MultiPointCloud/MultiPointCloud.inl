/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


inline MultiPointCloud::MultiPointCloud(std::string id, const GUI::Property::Paths& paths) : Visualizer(id), m_paths(paths), m_cloud(new Cloud()) {
}

inline MultiPointCloud::~MultiPointCloud() {
}

inline void MultiPointCloud::init() {
	addProperties();
	registerEvents();
	addClouds(m_paths);
}

inline void MultiPointCloud::render() {
	if (!m_cloud || !m_cloud->size()) return;

	auto mvMatrix = fw()->transforms()->modelview();
	auto prMatrix = fw()->transforms()->projection();
	m_rf["Main Cloud"]->render(mvMatrix, prMatrix);
	m_rf["Main Cloud Normals"]->render(mvMatrix, prMatrix);
	for (const auto& cloud : m_rf) {
		std::string name = cloud.first;
		if (name != "Main Cloud" && name != "Main Cloud Normals") cloud.second->render(mvMatrix, prMatrix);
	}
}

inline void MultiPointCloud::addProperties() {
	auto tree = gui()->properties()->add<Tree>("Visibility", "visibility");
	tree->setCallback([&] (std::string id, bool state) {
		if (m_rf.find(id) == m_rf.end()) return;
		m_rf[id]->setVisible(state);
	});

	auto groupCloud = gui()->properties()->add<Section>("Import/Export", "groupCloud");
	auto importFiles = groupCloud->add<Files>("Import Clouds", "importFiles");
	importFiles->setExtensions({"pcd"});
	importFiles->setCallback([&] (const GUI::Property::Paths& paths) { addClouds(paths); });
	auto exportFile = groupCloud->add<File>("Export Cloud", "exportFile");
	exportFile->setMode(File::SAVE);
	exportFile->setCallback([&] (const fs::path& path) { exportCloud(path); });
	groupCloud->collapse();

	auto groupRendering = gui()->properties()->add<Section>("Rendering", "groupRendering");
	auto thickness = groupRendering->add<Range>("Element Thickness", "thickness");
	thickness->setDigits(0);
	thickness->setMin(1);
	thickness->setMax(6);
	thickness->setValue(1);
	thickness->setCallback([&] (float value) { for (auto v : m_rf) v.second->setThickness(static_cast<int>(value)); });
	groupRendering->collapse();

	auto groupEdit = gui()->properties()->add<Section>("Edit Cloud", "groupEdit");
	auto diamFactor = groupEdit->add<Number>("Diameter Factor", "diamFactor");
	diamFactor->setMin(0.0001);
	diamFactor->setMax(1.0000);
	diamFactor->setValue(0.0016);
	diamFactor->setDigits(4);
	groupEdit->add<Button>("Resample")->setCallback([&] () { resample(); });
	groupEdit->collapse();
}

inline void MultiPointCloud::registerEvents() {
}

inline void MultiPointCloud::addClouds(const GUI::Property::Paths& paths) {
	if (!paths.size()) return;
	for (const auto& p : paths) {
		if (!fs::exists(p)) {
			gui()->log()->error("File \""+p.string()+"\" does not exist. Skipping this file.");
			continue;
		}
		Cloud::Ptr singleCloud = Tools::loadPointCloud(p);
		gui()->log()->verbose("Loaded point cloud with "+lexical_cast<std::string>(singleCloud->size())+" points.");
		m_cloud->insert(m_cloud->end(), singleCloud->begin(), singleCloud->end());
	}
	addCloud("Main Cloud", rgbaGrey(), m_cloud);
	addNormals("Main Cloud Normals", rgbaWhite(), m_cloud, false);
}

inline Rendered::Cloud::Ptr MultiPointCloud::addCloud(std::string name, RGBA color, const std::vector<Vector3f>& points, bool visible) {
	int thickness = gui()->properties()->get<Range>({"groupRendering", "thickness"})->value();
	RC::Ptr rc(new RC(color, thickness));
	rc->set(points);
	rc->setVisible(visible);
	auto tree = gui()->properties()->get<Tree>(path("visibility"));
	tree->add(name, {name}, visible);
	m_rf[name] = std::dynamic_pointer_cast<RF>(rc);
	return rc;
}

inline Rendered::Cloud::Ptr MultiPointCloud::addCloud(std::string name, RGBA color, Cloud::Ptr cloud, bool visible) {
	int thickness = gui()->properties()->get<Range>({"groupRendering", "thickness"})->value();
	RC::Ptr rc(new RC(color, thickness));
	try {
		rc->setFromPCLCloud(cloud->begin(), cloud->end());
	} catch (std::runtime_error& e) {
		gui()->log()->warn(e.what());
	}
	rc->setVisible(visible);
	auto tree = gui()->properties()->get<Tree>(path("visibility"));
	tree->add(name, {name}, visible);
	m_rf[name] = std::dynamic_pointer_cast<RF>(rc);
	return rc;
}

inline Rendered::Vectors::Ptr MultiPointCloud::addNormals(std::string name, RGBA color, Cloud::Ptr cloud, bool visible) {
	int thickness = gui()->properties()->get<Range>({"groupRendering", "thickness"})->value();
	RV::Ptr rv(new RV(color, thickness));
	rv->setFromPCLCloudNormals(cloud->begin(), cloud->end(), 0.1f);
	rv->setVisible(visible);
	auto tree = gui()->properties()->get<Tree>(path("visibility"));
	tree->add(name, {name}, visible);
	m_rf[name] = std::dynamic_pointer_cast<RF>(rv);
	return rv;
}

inline Rendered::Lines::Ptr MultiPointCloud::addLines(std::string name, RGBA color, const std::vector<Vector3f>& points, bool visible) {
	int thickness = gui()->properties()->get<Range>({"groupRendering", "thickness"})->value();
	RL::Ptr rl(new RL(color, thickness));
	rl->set(points);
	rl->setVisible(visible);
	auto tree = gui()->properties()->get<Tree>(path("visibility"));
	tree->add(name, {name}, visible);
	m_rf[name] = std::dynamic_pointer_cast<RF>(rl);
	return rl;
}

inline void MultiPointCloud::removeField(std::string name) {
	auto findIt = m_rf.find(name);
	if (findIt == m_rf.end()) return;
	auto tree = gui()->properties()->get<Tree>(path("visibility"));
	tree->remove(name);
	m_rf.erase(findIt);
}

inline void MultiPointCloud::exportCloud(const fs::path& path) {
	pcl::io::savePCDFileBinary(path.string(), *m_cloud);
	gui()->log()->info("Saved pointcloud to: \""+path.string()+"\"");
}

inline void MultiPointCloud::resample() {
	auto diamFactor = gui()->properties()->get<Number>({"groupEdit", "diamFactor"})->value();
	pcl::UniformSampling<Point> us;
	us.setInputCloud(m_cloud);
	us.setRadiusSearch(diamFactor * Tools::diameter(m_cloud));
	pcl::PointCloud<int> subsampled_indices;
	us.compute(subsampled_indices);
	std::sort(subsampled_indices.points.begin (), subsampled_indices.points.end ());
	Cloud::Ptr result(new Cloud());
	pcl::copyPointCloud(*m_cloud, subsampled_indices.points, *result);
	m_cloud = result;
	addCloud("Main Cloud", rgbaGrey(), m_cloud);
	addNormals("Main Cloud Normals", rgbaWhite(), m_cloud, false);
	gui()->log()->verbose("Resampled with diameter "+lexical_cast<std::string>(diamFactor)+".");
}
