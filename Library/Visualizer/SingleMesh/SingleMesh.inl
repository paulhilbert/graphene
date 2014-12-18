inline SingleMesh::SingleMesh(std::string id, fs::path meshFile) : Visualizer(id), m_meshFile(meshFile) {
}

inline SingleMesh::~SingleMesh() {
}

inline void SingleMesh::init() {
    if (!fs::exists(m_meshFile)) {
        gui()->log()->error("Mesh file \"" + m_meshFile.string() + "\" does not exist.");
        return;
    }
    m_mesh = std::make_shared<RenderedMeshT>(m_meshFile.string(), false);
    m_mesh->init();
    addObject("main mesh", m_mesh);
	gui()->log()->info("Loaded mesh with "+lexical_cast<std::string>(m_mesh->mesh()->n_vertices())+" vertices and "+lexical_cast<std::string>(m_mesh->mesh()->n_faces())+" faces.");

	addProperties();
	addModes();
	registerEvents();
}

inline void SingleMesh::addProperties() {
}

inline void SingleMesh::addModes() {
	auto showGroup = gui()->modes()->addGroup("showGroup");
	showGroup->addOption("showClip", "Enable Clipping", std::string(ICON_PREFIX)+"clipping.png");
	auto editGroup = gui()->modes()->addGroup("editGroup");
	editGroup->addOption("editClip", "Modify Clipping Plane", std::string(ICON_PREFIX)+"clipplane.png");

    showGroup->setCallback([&] (std::string option, bool state) { if (option == "showClip") m_mesh->set_clipping(state); });
}

inline void SingleMesh::registerEvents() {
    fw()->events()->connect<void (int, int, int, int)>("LEFT_DRAG", [&] (int, int dy, int, int) {
        if (!gui()->modes()->group("editGroup")->option("editClip")->active()) return;
        m_mesh->delta_clipping_height(-dy * 0.01f);
    });
}
