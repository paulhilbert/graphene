inline NormalField::NormalField(RGBA baseColor, RenderKernel::Ptr kernel) : Field(baseColor, kernel) {
}

inline NormalField::~NormalField() {
}

inline void NormalField::set(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals) {
	if (points.size() != normals.size()) throw std::runtime_error("Mismatch of point and normal count.");

	m_pointCount = static_cast<unsigned int>(points.size());
	m_colors = Annotation::Colors(m_pointCount, m_color);
	m_geometry.reset();
	m_geometry = std::shared_ptr<Buffer::Geometry>(new Buffer::Geometry());
	m_geometry->init();
	m_geometry->addVertices(points);
	m_geometry->addNormals(normals);
	m_geometry->addColors(m_colors);
	m_geometry->upload();
	m_geometry->enableVertices();
	m_geometry->enableNormals();
	m_geometry->enableColors();
	m_geometry->bindVertices(m_kernel->program(), "position");
	m_geometry->bindNormals(m_kernel->program(), "normals");
	m_geometry->bindColors(m_kernel->program(), "color");
}

inline void NormalField::render(const Eigen::Matrix4f& mvMatrix, const Eigen::Matrix4f& prMatrix, const Eigen::Matrix3f& nmMatrix) {
	if (!m_visible || !m_geometry) return;
	m_kernel->program().use();
	m_kernel->program().setUniformMat4("mvM", mvMatrix.data());
	m_kernel->program().setUniformMat4("prM", prMatrix.data());
	m_kernel->program().setUniformMat3("nmM", nmMatrix.data());
	//m_kernel->program().setUniformVec4("color", &m_color[0]);

	// store blend mode and enable blending
	GLboolean blendEnabled;
	glGetBooleanv(GL_BLEND, &blendEnabled);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// render
	m_geometry->bind();
	m_kernel->renderElements(m_pointCount);
	m_geometry->release();

	// restore blend mode
	if (!blendEnabled) glDisable(GL_BLEND);
}

inline void NormalField::renderHDR(const Eigen::Matrix4f& mvMatrix, const Eigen::Matrix4f& prMatrix, const Eigen::Matrix3f&, FW::EnvTex envTex, float specularity, const Eigen::Vector3f& viewDir) {
	if (!m_visible || !m_geometry) return;
	m_kernel->programHDR().use();
	m_kernel->programHDR().setUniformMat4("mvM", mvMatrix.data());
	m_kernel->programHDR().setUniformMat4("prM", prMatrix.data());
	//m_kernel->program().setUniformMat3("nmM", nmMatrix.data());
	m_kernel->programHDR().setUniformVec3("viewDir", viewDir.data());
	m_kernel->programHDR().setUniformVar1f("specularity", specularity);
	//m_kernel->program().setUniformVec4("color", &m_color[0]);

	// store blend mode and enable blending
	GLboolean blendEnabled;
	glGetBooleanv(GL_BLEND, &blendEnabled);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glActiveTexture(GL_TEXTURE0);
	envTex.diffuse->bind();
	glActiveTexture(GL_TEXTURE1);
	envTex.specular->bind();

	// render
	m_geometry->bind();
	m_kernel->renderElementsHDR(m_pointCount);
	m_geometry->release();

	glActiveTexture(GL_TEXTURE0);
	Buffer::Texture::unbind();
	glActiveTexture(GL_TEXTURE1);
	Buffer::Texture::unbind();

	// restore blend mode
	if (!blendEnabled) glDisable(GL_BLEND);
}
