inline SinglePointCloud::SinglePointCloud(std::string id, const GUI::Property::Paths& paths) : Visualizer(id), m_paths(paths), m_cloud(new Cloud()) {
}

inline SinglePointCloud::~SinglePointCloud() {
}

inline void SinglePointCloud::init() {
	addProperties();
	registerEvents();
	m_rendered = Rendered::Cloud::Ptr(new Rendered::Cloud(Eigen::Vector4f(0.4f, 0.4f, 0.4f, 1.f)));
	addClouds(m_paths);
	uploadCloud();
}

inline void SinglePointCloud::render() {
	m_rendered->render(fw()->transforms()->modelview(), fw()->transforms()->projection());
}

inline void SinglePointCloud::addProperties() {
	auto groupCloud = gui()->properties()->add<Section>("Import/Export", "groupCloud");
	auto importFiles = groupCloud->add<Files>("Import Clouds", "importFiles");
	importFiles->setExtensions({"pcd"});
	importFiles->setCallback([&] (const GUI::Property::Paths& paths) { addClouds(paths); });
	auto exportFile = groupCloud->add<File>("Export Cloud", "exportFile");
	exportFile->setMode(File::SAVE);
	exportFile->setCallback([&] (const fs::path& path) { exportCloud(path); });
	groupCloud->collapse();

	auto groupEdit = gui()->properties()->add<Section>("Edit Cloud", "groupEdit");
	auto diamFactor = groupEdit->add<Number>("Diameter Factor", "diamFactor");
	diamFactor->setMin(0.0001);
	diamFactor->setMax(1.0000);
	diamFactor->setValue(0.0016);
	diamFactor->setDigits(4);
	groupEdit->add<Button>("Resample")->setCallback([&] () { resample(); });
	groupEdit->collapse();
}

inline void SinglePointCloud::registerEvents() {
}

inline void SinglePointCloud::addClouds(const GUI::Property::Paths& paths) {
	for (const auto& p : paths) {
		if (!fs::exists(p)) {
			gui()->log()->error("File \""+p.string()+"\" does not exist. Skipping this file.");
			continue;
		}
		if (p.extension() != ".pcd") {
			gui()->log()->error("File \""+p.string()+"\" is not a .pcd file. Skipping this file.");
			continue;
		}
		Cloud singleCloud;
		if (pcl::io::loadPCDFile<Point>(p.string(), singleCloud) == -1) {
			gui()->log()->error("Couldn't read file \""+p.string()+"\". Skipping this file.");
			continue;
		}
		gui()->log()->verbose("Loaded point cloud with "+lexical_cast<std::string>(singleCloud.size())+" points.");
		m_cloud->insert(m_cloud->end(), singleCloud.begin(), singleCloud.end());
	}
	uploadCloud();
}

inline void SinglePointCloud::exportCloud(const fs::path& path) {
	pcl::io::savePCDFileBinary(path.string(), *m_cloud);
	gui()->log()->info("Saved pointcloud to: \""+path.string()+"\"");
}

inline void SinglePointCloud::uploadCloud() {
	m_rendered->setFromPCLCloud(m_cloud->begin(), m_cloud->end());
}

inline void SinglePointCloud::resample() {
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
	uploadCloud();
	gui()->log()->verbose("Resampled with diameter "+lexical_cast<std::string>(diamFactor)+".");
}
