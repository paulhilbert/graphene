inline SingleMesh::SingleMesh(std::string id, fs::path meshFile) : Visualizer(id), m_meshFile(meshFile) {
}

inline SingleMesh::~SingleMesh() {
}

inline void SingleMesh::init() {
	addProperties();
	addModes();

    omerr().disable();
	OpenMesh::IO::Options opt;
	opt += OpenMesh::IO::Options::FaceNormal;
	opt += OpenMesh::IO::Options::VertexNormal;
	opt += OpenMesh::IO::Options::VertexColor;
	m_mesh = std::make_shared<MeshT>();
	m_mesh->request_vertex_colors();
	m_mesh->request_vertex_normals();
	m_mesh->request_face_normals();
    if (!OpenMesh::IO::read_mesh(*m_mesh, m_meshFile.string(), opt)) {
        gui()->log()->error("Unable to read mesh file.");
        return;
    }
    m_mesh->triangulate();
    if (!opt.face_has_normal()) {
        m_mesh->update_face_normals();
    }
    m_mesh->update_normals();
    for (auto it = m_mesh->vertices_begin(); it != m_mesh->vertices_end(); ++it) {
        m_mesh->set_color(*it, ColorT(1.f, 1.f, 1.f, 1.f));
        m_bbox.extend(Eigen::Vector3f(m_mesh->point(*it).data()));
    }
	gui()->log()->info("Loaded mesh with "+lexical_cast<std::string>(m_mesh->n_vertices())+" vertices and "+lexical_cast<std::string>(m_mesh->n_faces())+" faces.");

	m_rm = std::shared_ptr<RenderedMeshT>(new RenderedMeshT(m_mesh, false));

	registerEvents();
}

inline void SingleMesh::initGeometry(harmont::shader_program::ptr program, harmont::pass_type_t type) {
    m_rm->init(program, type);
}

inline void SingleMesh::render(harmont::shader_program::ptr program, harmont::pass_type_t type) {
	//bool clipping = gui()->modes()->group("showGroup")->option("showClip")->active();
	//if (clipping) {
		//Eigen::Vector3f clipNormal = Eigen::Vector3f(0, 0, 1);
		//program.setUniformVec3("clipNormal", clipNormal.data());
		//program.setUniformVar1f("clipDistance", m_clippingHeight);
		//glEnable(GL_CLIP_DISTANCE0);
	//}

	m_rm->render(program, type);

	//if (clipping) {
		//glDisable(GL_CLIP_DISTANCE0);
	//}
}

inline void SingleMesh::addProperties() {
}

inline void SingleMesh::addModes() {
	auto showGroup = gui()->modes()->addGroup("showGroup");
	showGroup->addOption("showClip", "Enable Clipping", std::string(ICON_PREFIX)+"clipping.png");
	auto editGroup = gui()->modes()->addGroup("editGroup");
	editGroup->addOption("editClip", "Modify Clipping Plane", std::string(ICON_PREFIX)+"clipplane.png");
}

inline void SingleMesh::registerEvents() {
	//m_clipRangeMin =  std::numeric_limits<float>::infinity();
	//m_clipRangeMax = -std::numeric_limits<float>::infinity();
	//for (auto vertexId : Traits::vertices(*m_mesh)) {
		//float vertexHeight = Traits::vertexPosition(*m_mesh, vertexId)[2];
		//if (vertexHeight < m_clipRangeMin) m_clipRangeMin = vertexHeight;
		//if (vertexHeight > m_clipRangeMax) m_clipRangeMax = vertexHeight;
	//}
	//m_clipRangeMin -= 0.1f;
	//m_clipRangeMax += 0.1f;
	//m_clippingHeight = m_clipRangeMin + 0.5f * (m_clipRangeMax - m_clipRangeMin);
	//fw()->events()->connect<void (int, int, int, int)>("LEFT_DRAG", [&] (int, int dy, int, int) {
		//if (!gui()->modes()->group("editGroup")->option("editClip")->active()) return;
		//m_clippingHeight -= 0.05f * static_cast<float>(dy);
		//if (m_clippingHeight < m_clipRangeMin) m_clippingHeight = m_clipRangeMin;
		//if (m_clippingHeight > m_clipRangeMax) m_clippingHeight = m_clipRangeMax;
	//});
}

inline BoundingBox SingleMesh::boundingBox() const {
	return m_bbox;
}
