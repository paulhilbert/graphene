inline PointCloudEditor::PointCloudEditor(std::string id, const Paths& paths) : Visualizer(id), SinglePointCloud(id, paths), SelectModes<Cloud,Point>(id, m_cloud) {
}

inline PointCloudEditor::~PointCloudEditor() {
}

inline void PointCloudEditor::init() {
	SinglePointCloud::init();
	SelectModes::init();
	addProperties();
	registerEvents();
}

inline void PointCloudEditor::render() {
	SinglePointCloud::render();
	SelectModes::render();
}

inline void PointCloudEditor::addProperties() {
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
