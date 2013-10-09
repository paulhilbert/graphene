template <class Entities, class Entity>
inline SelectModes<Entities, Entity>::SelectModes(std::string id, EntitiesPtr entities, Methods exclude) : Visualizer(id), m_entities(entities), m_exclude(exclude), m_activeMethod(METHOD_NONE) {
}

template <class Entities, class Entity>
inline SelectModes<Entities, Entity>::~SelectModes() {
}

template <class Entities, class Entity>
inline void SelectModes<Entities, Entity>::init(const Eigen::Vector4f& color) {
	addProperties();
	registerEvents();
	addModes();

	if (!(m_exclude & METHOD_AREA)) {
		m_areaSelect = Input::AreaSelect::Ptr(new Input::AreaSelect(fw()));
		m_areaSelect->setStopCallback([&] () {
			if (!m_entities) return;
			if (!resetSelectionRender()) return;
			int idx = 0;
			IdxSet selection;
			for (const Entity& entity : *m_entities) {
				if (isInsideSelection(entity, std::dynamic_pointer_cast<Input::SelectionMethod>(m_areaSelect), m_activeMethod)) {
					selection.push_back(idx);
				}
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
			if (m_selection.size()) updateSelectionRender(m_selection);
		});
		m_areaSelect->setUnselectCallback([&] () { resetSelection(); });
		m_areaSelect->init(color);
	}

	if (!(m_exclude & METHOD_PAINT)) {
		m_paintSelect = Input::PaintSelect::Ptr(new Input::PaintSelect(fw(), 10));
		m_paintSelect->setStartCallback([&] () { 
			if (!fw()->modifier()->shift() && !fw()->modifier()->ctrl()) {
				resetSelection();
			}
		});
		m_paintSelect->setDragCallback([&] () {
			if (!m_entities) return;
			if (!resetSelectionRender()) return;
			int idx = 0;
			IdxSet selection;
			for (const Entity& entity : *m_entities) {
				if (isInsideSelection(entity, std::dynamic_pointer_cast<Input::SelectionMethod>(m_paintSelect), m_activeMethod)) {
					selection.push_back(idx);
				}
				++idx;
			}
			if (fw()->modifier()->ctrl()) {
				if (!m_selection.size()) return;
				m_selection = Algorithm::setDifference(m_selection, selection);
			} else {
				m_selection = Algorithm::setUnion(m_selection, selection);
			}
			if (m_selection.size()) updateSelectionRender(m_selection);
		});
		m_paintSelect->setStopCallback([&] () {
			if (!m_entities) return;
			if (!resetSelectionRender()) return;
			int idx = 0;
			IdxSet selection;
			for (const Entity& entity : *m_entities) {
				if (isInsideSelection(entity, std::dynamic_pointer_cast<Input::SelectionMethod>(m_paintSelect), m_activeMethod)) {
					selection.push_back(idx);
				}
				++idx;
			}
			if (fw()->modifier()->ctrl()) {
				if (!m_selection.size()) return;
				m_selection = Algorithm::setDifference(m_selection, selection);
			} else {
				m_selection = Algorithm::setUnion(m_selection, selection);
			}
			if (m_selection.size()) updateSelectionRender(m_selection);
		});
		m_paintSelect->setUnselectCallback([&] () { resetSelection(); });
		m_paintSelect->init(color);
	}
}

template <class Entities, class Entity>
inline void SelectModes<Entities, Entity>::render() {
	m_areaSelect->render();
	m_paintSelect->render();
}

template <class Entities, class Entity>
inline void SelectModes<Entities, Entity>::addProperties() {
}

template <class Entities, class Entity>
inline void SelectModes<Entities, Entity>::registerEvents() {
}

template <class Entities, class Entity>
inline void SelectModes<Entities, Entity>::addModes() {
	bool area = !(m_exclude & METHOD_AREA);
	bool paint = !(m_exclude & METHOD_PAINT);

	auto groupSel = gui()->modes()->addGroup("selection");
	if (area)  groupSel->addOption("selArea", "Select Area", "Icons/area_select.png");
	if (paint) groupSel->addOption("selPaint", "Select By Painting", "Icons/paint_select.png");
	groupSel->setCallback([&] (std::string mode, bool active) {
		if (active) {
			if (mode == "selArea")  m_areaSelect->enable();
			if (mode == "selPaint") m_paintSelect->enable();
		} else {
			if (mode == "selArea")  m_areaSelect->disable();
			if (mode == "selPaint") m_paintSelect->disable();
		}
		if (gui()->modes()->group("selection")->getCurrentOption() == "selArea")        m_activeMethod = METHOD_AREA;
		else if (gui()->modes()->group("selection")->getCurrentOption() == "selPaint")  m_activeMethod = METHOD_PAINT;
		else m_activeMethod = METHOD_NONE;
	});
}

template <class Entities, class Entity>
inline void SelectModes<Entities,Entity>::resetSelection() {
	m_selection.clear();
	resetSelectionRender();
}
