#include "MergeAndEditClouds.h"

#include <Vis/Colors.h>
using namespace Vis::Colors;

#include <pcl/keypoints/uniform_sampling.h>

namespace FW {

MergeAndEditClouds::MergeAndEditClouds(std::string id, const GUI::Property::Paths& paths) : Visualizer(id), m_paths(paths), m_cloud(new Cloud()) {
}

MergeAndEditClouds::~MergeAndEditClouds() {
}

void MergeAndEditClouds::init() {
	addProperties();
	addModes();
	registerEvents();

	m_rendered = Rendered::Cloud::Ptr(new Rendered::Cloud(Eigen::Vector4f(0.4f, 0.4f, 0.4f, 1.f)));

	setupSelection();

	addClouds(m_paths);
	uploadCloud();
}

void MergeAndEditClouds::render() {
	m_rendered->render(fw()->transforms()->modelview(), fw()->transforms()->projection());
	m_areaSelect->render();
	m_paintSelect->render();
}

void MergeAndEditClouds::addProperties() {
	auto groupCloud = gui()->properties()->add<Group>("Import/Export", "groupCloud");
	auto importFiles = groupCloud->add<Files>("Import Clouds", "importFiles");
	importFiles->setExtensions({"pcd"});
	importFiles->setCallback([&] (const GUI::Property::Paths& paths) { addClouds(paths); });
	auto exportFile = groupCloud->add<File>("Export Cloud", "exportFile");
	exportFile->setMode(File::SAVE);
	exportFile->setCallback([&] (const fs::path& path) { exportCloud(path); });

	auto groupEditSel = gui()->properties()->add<Group>("Edit Selection", "groupEditSel");
	auto crop = groupEditSel->add<Button>("Crop Selection", "crop");
	crop->setCallback(std::bind(&MergeAndEditClouds::crop, this));
	crop->disable();
	auto erase = groupEditSel->add<Button>("Erase Selection", "erase");
	erase->setCallback(std::bind(&MergeAndEditClouds::erase, this));
	erase->disable();

	auto groupEdit = gui()->properties()->add<Group>("Edit Cloud", "groupEdit");
	auto diamFactor = groupEdit->add<Number>("Diameter Factor", "diamFactor");
	diamFactor->setMin(0.0001);
	diamFactor->setMax(1.0000);
	diamFactor->setValue(0.0016);
	diamFactor->setDigits(4);
	groupEdit->add<Button>("Resample")->setCallback([&] () { resample(); });
}

void MergeAndEditClouds::addModes() {
	auto groupSel = gui()->modes()->addGroup("selection");
	auto areaSel = groupSel->addOption("selArea", "Select Area", "Icons/area_select.png");
	auto selPaint = groupSel->addOption("selPaint", "Select By Painting", "Icons/paint_select.png");
	groupSel->setCallback([&] (std::string mode, bool active) {
		if (mode == "selArea" && active) {
			m_areaSelect->enable();
		} else {
			m_areaSelect->disable();
		}
		if (mode == "selPaint" && active) {
			m_paintSelect->enable();
		} else {
			m_paintSelect->disable();
		}
	});
}

void MergeAndEditClouds::registerEvents() {
	fw()->events()->connect<void (FW::Events::Keys::Special)>("SPECIAL_KEY_RELEASE", [&] (FW::Events::Keys::Special key) {
		if (key == FW::Events::Keys::DEL) erase();
		if (key == FW::Events::Keys::INSERT) crop();
	});
}

void MergeAndEditClouds::addClouds(const GUI::Property::Paths& paths) {
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

void MergeAndEditClouds::exportCloud(const fs::path& path) {
	pcl::io::savePCDFileBinary(path.string(), *m_cloud);
	gui()->log()->info("Saved pointcloud to: \""+path.string()+"\"");
}

void MergeAndEditClouds::uploadCloud() {
	m_rendered->setFromPCLCloud(m_cloud->begin(), m_cloud->end());
}

void MergeAndEditClouds::setupSelection() {
	m_areaSelect = Input::AreaSelect::Ptr(new Input::AreaSelect(fw()));
	//m_areaSelect->setStartCallback(std::bind(&MergeAndEditClouds::resetSelection, this));
	m_areaSelect->setStopCallback([&] () {
		if (!m_rendered) return;
		m_rendered->clearAnnotations();
		int idx = 0;
		IdxSet selection;
		for (const auto& pnt : *m_cloud) {
			if (m_areaSelect->pointInSelection(pnt.getVector3fMap())) selection.push_back(idx);
			++idx;
		}
		if (fw()->modifier()->ctrl()) {
			if (!m_selection.size()) return;
			m_selection = Algorithm::setDifference(m_selection, selection);
		} else if (fw()->modifier()->shift()) {
			m_selection = Algorithm::setUnion(m_selection, selection);
		} else {
			m_selection = selection;
		}
		if (m_selection.size()) {
			m_rendered->annotate(m_selection, "selection")->colorize(rgbaWhite());
			gui()->properties()->get<Button>({"groupEditSel", "crop"})->enable();
			gui()->properties()->get<Button>({"groupEditSel", "erase"})->enable();
		}
	});
	m_areaSelect->setUnselectCallback(std::bind(&MergeAndEditClouds::resetSelection, this));
	m_areaSelect->init(Eigen::Vector4f(0.f, 0.4f, 1.f, 0.4f));

	m_paintSelect = Input::PaintSelect::Ptr(new Input::PaintSelect(fw(), 10));
	m_paintSelect->setStartCallback([&] () { if (!fw()->modifier()->shift() && !fw()->modifier()->ctrl()) resetSelection(); });
	m_paintSelect->setDragCallback([&] () {
		if (!m_rendered) return;
		m_rendered->clearAnnotations();
		int idx = 0;
		IdxSet selection;
		for (const auto& pnt : *m_cloud) {
			if (m_paintSelect->pointInSelection(pnt.getVector3fMap())) selection.push_back(idx);
			++idx;
		}
		if (fw()->modifier()->ctrl()) {
			if (!m_selection.size()) return;
			m_selection = Algorithm::setDifference(m_selection, selection);
		} else {
			m_selection = Algorithm::setUnion(m_selection, selection);
		}
		if (m_selection.size()) {
			m_rendered->annotate(m_selection, "selection")->colorize(rgbaWhite());
			gui()->properties()->get<Button>({"groupEditSel", "crop"})->enable();
			gui()->properties()->get<Button>({"groupEditSel", "erase"})->enable();
		}
	});
	m_paintSelect->setStopCallback([&] () {
		if (!m_rendered) return;
		m_rendered->clearAnnotations();
		int idx = 0;
		IdxSet selection;
		for (const auto& pnt : *m_cloud) {
			if (m_paintSelect->pointInSelection(pnt.getVector3fMap())) selection.push_back(idx);
			++idx;
		}
		if (fw()->modifier()->ctrl()) {
			if (!m_selection.size()) return;
			m_selection = Algorithm::setDifference(m_selection, selection);
		} else {
			m_selection = Algorithm::setUnion(m_selection, selection);
		}
		if (m_selection.size()) {
			m_rendered->annotate(m_selection, "selection")->colorize(rgbaWhite());
			gui()->properties()->get<Button>({"groupEditSel", "crop"})->enable();
			gui()->properties()->get<Button>({"groupEditSel", "erase"})->enable();
		}
	});
	m_paintSelect->setUnselectCallback(std::bind(&MergeAndEditClouds::resetSelection, this));
	m_paintSelect->init(Eigen::Vector4f(0.f, 0.4f, 1.f, 0.4f));
}

void MergeAndEditClouds::resetSelection() {
	m_selection.clear();
	if (m_rendered) m_rendered->clearAnnotations();
	gui()->properties()->get<Button>({"groupEditSel", "crop"})->disable();
	gui()->properties()->get<Button>({"groupEditSel", "erase"})->disable();
}

void MergeAndEditClouds::crop() {
	if (!m_selection.size()) return;
	Cloud::Ptr newCloud(new Cloud(*m_cloud, m_selection));
	resetSelection();
	m_cloud = newCloud;
	m_rendered->setFromPCLCloud(m_cloud->begin(), m_cloud->end());
}

void MergeAndEditClouds::erase() {
	if (!m_selection.size()) return;
	IdxSet invert(m_cloud->size());
	std::iota(invert.begin(), invert.end(), 0);
	Algorithm::remove(invert, [&] (int idx) { return std::binary_search(m_selection.begin(), m_selection.end(), idx); });
	Cloud::Ptr newCloud(new Cloud(*m_cloud, invert));
	resetSelection();
	m_cloud = newCloud;
	m_rendered->setFromPCLCloud(m_cloud->begin(), m_cloud->end());
}

void MergeAndEditClouds::resample() {
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

MergeAndEditClouds::Factory::Factory() : FW::Factory() {
}

MergeAndEditClouds::Factory::~Factory() {
}

void MergeAndEditClouds::Factory::init() {
	auto paths = gui()->properties()->add<Files>("Input Files:", "paths");
	paths->setExtensions({"pcd"});
}

Visualizer::Ptr MergeAndEditClouds::Factory::addVisualizer() {
	std::string name = gui()->properties()->get<String>({"__name__"})->value();
	auto paths = gui()->properties()->get<Files>({"paths"})->value();
	MergeAndEditClouds::Ptr vis(new MergeAndEditClouds(name, paths));
	return std::dynamic_pointer_cast<Visualizer>(vis);
}

} // FW

