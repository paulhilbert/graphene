/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


inline SinglePointCloud::SinglePointCloud(std::string id, const GUI::Property::Paths& paths, std::string upAxis, float scale, bool recenter) : Visualizer(id), m_paths(paths), m_cloud(nullptr), m_upAxis(upAxis), m_scale(scale), m_recenter(recenter) {
}

inline SinglePointCloud::~SinglePointCloud() {
}

inline void SinglePointCloud::init() {
	addProperties();
    addModes();
	registerEvents();
	addClouds(m_paths);
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
}

inline void SinglePointCloud::addModes() {
	auto showGroup = gui()->modes()->addGroup("showGroup");
	showGroup->addOption("showClip", "Enable Clipping", std::string(ICON_PREFIX)+"clipping.png");
	auto editGroup = gui()->modes()->addGroup("editGroup");
	editGroup->addOption("editClip", "Modify Clipping Plane", std::string(ICON_PREFIX)+"clipplane.png");

    showGroup->setCallback([&] (std::string option, bool state) { if (option == "showClip") m_cloud->set_clipping(state); });
}

inline void SinglePointCloud::registerEvents() {
    fw()->events()->connect<void (int, int, int, int)>("LEFT_DRAG", [&] (int, int dy, int, int) {
        if (!gui()->modes()->group("editGroup")->option("editClip")->active()) return;
        m_cloud->delta_clipping_height(-dy * 0.01f);
    });
}

inline void SinglePointCloud::addClouds(const GUI::Property::Paths& paths) {
    boost::shared_ptr<CloudT> cloud = m_cloud ? m_cloud->cloud() : boost::make_shared<CloudT>();
	for (const auto& p : paths) {
		if (!fs::exists(p)) {
			gui()->log()->error("File \""+p.string()+"\" does not exist. Skipping this file.");
			continue;
		}
		try {
			std::vector<Vector4f> colors;
			Cloud::Ptr singleCloud = Tools::loadPointCloud(p, colors);
			gui()->log()->verbose("Loaded point cloud with "+lexical_cast<std::string>(singleCloud->size())+" points.");
			cloud->insert(cloud->end(), singleCloud->begin(), singleCloud->end());
		} catch (std::runtime_error& e) {
			gui()->log()->error(e.what());
			continue;
		}
	}
	//Tools::adjust(m_cloud, m_upAxis, m_scale, m_recenter);

    try {
        removeObject("main cloud");
    } catch(...) {
    }
    m_cloud = std::make_shared<RenderedCloudT>(cloud);
    m_cloud->init();
    addObject("main cloud", m_cloud);
}

inline void SinglePointCloud::exportCloud(const fs::path& path) {
	pcl::io::savePCDFileBinary(path.string(), *(m_cloud->cloud()));
	gui()->log()->info("Saved pointcloud to: \""+path.string()+"\"");
}
