inline SingleMesh::SingleMesh(std::string id, fs::path meshFile, std::string upAxis, std::string frontAxis, float scale, bool recenter) : Visualizer(id), m_meshFile(meshFile), m_upAxis(upAxis), m_frontAxis(frontAxis), m_scale(scale), m_recenter(recenter) {
}

inline SingleMesh::~SingleMesh() {
}

inline void SingleMesh::init() {
	addProperties();
	addModes();

	m_mesh = std::make_shared<Mesh>();
	if (!Traits::loadFromFile(*m_mesh, m_meshFile.string())) {
		gui()->log()->error("Could not open mesh file \""+m_meshFile.string()+"\"");
	}
	gui()->log()->info("Loaded mesh with "+lexical_cast<std::string>(Traits::numVertices(*m_mesh))+" vertices and "+lexical_cast<std::string>(Traits::numFaces(*m_mesh))+" faces.");
	Traits::adjust(*m_mesh, m_upAxis, m_frontAxis, m_scale, m_recenter);

	m_program = std::make_shared<ShaderProgram>();
	m_program->addShaders(std::string(GLSL_PREFIX)+"mesh.vert", std::string(GLSL_PREFIX)+"mesh.frag");
	m_program->link();
	Eigen::Vector3f lightDir(1.f, 1.f, 1.f);
	lightDir.normalize();
	m_program->use();
	m_program->setUniformVec3("lightDir", lightDir.data());

	m_rm = std::make_shared<Rendered::Mesh<Mesh>>(m_mesh, m_program, false);

	registerEvents();
}

inline void SingleMesh::render() {
	if (!m_visible) return;
	m_program->use();
	
	bool clipping = gui()->modes()->group("showGroup")->option("showClip")->active();
	if (clipping) {
		Eigen::Vector3f clipNormal = Eigen::Vector3f(0, 0, 1);
		m_program->setUniformVec3("clipNormal", clipNormal.data());
		m_program->setUniformVar1f("clipDistance", m_clippingHeight);
		glEnable(GL_CLIP_DISTANCE0);
	}
	glEnable(GL_DEPTH_TEST);
	
	m_program->setUniformMat4("mvM", fw()->transforms()->modelview().data());
	m_program->setUniformMat4("prM", fw()->transforms()->projection().data());
	m_program->setUniformMat3("nmM", fw()->transforms()->normal().data());
	m_rm->render();
	
	glDisable(GL_DEPTH_TEST);
	if (clipping) {
		glDisable(GL_CLIP_DISTANCE0);
	}
}

inline void SingleMesh::addProperties() {
	auto visible = gui()->properties()->add<Bool>("Render Mesh", "visible");
	visible->setValue(true);
	visible->setCallback([&] (bool value) { m_visible = value; });
}

inline void SingleMesh::addModes() {
	auto renderGroup = gui()->modes()->addGroup("renderGroup");
	renderGroup->addOption("smooth", "Smooth Normal Interpolation", std::string(ICON_PREFIX)+"smooth.png");
	renderGroup->setCallback([&] (std::string option, bool state) { if (option == "smooth") m_rm->setSmoothNormals(state); });

	auto showGroup = gui()->modes()->addGroup("showGroup");
	showGroup->addOption("showClip", "Enable Clipping", std::string(ICON_PREFIX)+"clipping.png");
	auto editGroup = gui()->modes()->addGroup("editGroup");
	editGroup->addOption("editClip", "Modify Clipping Plane", std::string(ICON_PREFIX)+"clipplane.png");
}

inline void SingleMesh::registerEvents() {
	m_clipRangeMin =  std::numeric_limits<float>::infinity();
	m_clipRangeMax = -std::numeric_limits<float>::infinity();
	for (auto vertexId : Traits::vertices(*m_mesh)) {
		float vertexHeight = Traits::vertexPosition(*m_mesh, vertexId)[2];
		if (vertexHeight < m_clipRangeMin) m_clipRangeMin = vertexHeight;
		if (vertexHeight > m_clipRangeMax) m_clipRangeMax = vertexHeight;
	}
	m_clipRangeMin -= 0.1f;
	m_clipRangeMax += 0.1f;
	m_clippingHeight = m_clipRangeMin + 0.5f * (m_clipRangeMax - m_clipRangeMin);
	fw()->events()->connect<void (int, int, int, int)>("LEFT_DRAG", [&] (int, int dy, int, int) {
		if (!gui()->modes()->group("editGroup")->option("editClip")->active()) return;
		m_clippingHeight -= 0.05f * static_cast<float>(dy);
		if (m_clippingHeight < m_clipRangeMin) m_clippingHeight = m_clipRangeMin;
		if (m_clippingHeight > m_clipRangeMax) m_clippingHeight = m_clipRangeMax;
	});
}
