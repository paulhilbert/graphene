/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


/// RENDERKERNEL ///

inline RenderKernel::RenderKernel() {
}

inline RenderKernel::~RenderKernel() {
}

inline void RenderKernel::init() {
	this->initShader();
	m_prog.link();
}

inline ShaderProgram& RenderKernel::program() {
	return m_prog;
}

inline const ShaderProgram& RenderKernel::program() const {
	return m_prog;
}

/// ANNOTATION ///

inline Annotation::Annotation(const std::vector<int>& indices, std::string name, Field* field) : m_indices(indices), m_name(name), m_field(field) {
}

inline Annotation::~Annotation() {
}

inline const std::vector<int>& Annotation::indices() const {
	return m_indices;
}

inline std::vector<int>& Annotation::indices() {
	return m_indices;
}

inline void Annotation::remove() {
	m_field->removeAnnotation(m_name);
}


inline void Annotation::colorize(Color color) {
	Colors colors(m_indices.size(), color);
	colorize(colors);
}

inline void Annotation::colorize(const Colors& colors) {
	asserts(m_indices.size() == colors.size(), "Invalid color array size");
	Colors& oldColors = m_field->getColors();
	for (unsigned int i = 0; i < m_indices.size(); ++i) {
		oldColors[m_indices[i]] = colors[i];
	}
	m_field->upload();
}

inline void Annotation::colorize(const CAssign& colorMap) {
	Colors& colors = m_field->getColors();
	for (const auto& idx : m_indices) colors[idx] = colorMap(idx);
	m_field->upload();
}

inline void Annotation::colorize(const Scalars& scalars, const CMap&  colorMap) {
	asserts(scalars.size() == m_indices.size(), "Invalid scalar array size");
	auto& colors = m_field->getColors();
	for (unsigned int i = 0; i < m_indices.size(); ++i) colors[m_indices[i]] = colorMap(scalars[i]);
	m_field->upload();
}

inline void Annotation::colorize(const ScalarField& field, const CMap&  colorMap) {
	auto& colors = m_field->getColors();
	for (unsigned int i = 0; i < m_indices.size(); ++i) colors[m_indices[i]] = colorMap(field(m_indices[i]));
	m_field->upload();
}




/// RENDEREDFIELD ///

inline Field::Field(Annotation::Color baseColor, RenderKernel::Ptr kernel) : m_color(baseColor), m_visible(true), m_kernel(kernel) {
	m_kernel->init();
}

inline Field::~Field() {
}

inline void Field::setVisible(bool visible) {
	m_visible = visible;
}

inline bool Field::getVisible() const {
	return m_visible;
}

inline void Field::set(const std::vector<Eigen::Vector3f>& points) {
	m_pointCount = static_cast<unsigned int>(points.size());
	m_colors = Annotation::Colors(m_pointCount, m_color);
	m_geometry.reset();
	m_geometry = std::shared_ptr<Buffer::Geometry>(new Buffer::Geometry());
	m_geometry->init();
	m_geometry->addVertices(points);
	m_geometry->addColors(m_colors);
	m_geometry->upload();
	m_geometry->enableVertices();
	m_geometry->enableColors();
	m_geometry->bindVertices(m_kernel->program(), "position");
	m_geometry->bindColors(m_kernel->program(), "color");
}

inline void Field::render(const Eigen::Matrix4f& mvMatrix, const Eigen::Matrix4f& prMatrix) {
	if (!m_visible || !m_geometry) return;
	m_kernel->program().use();
	m_kernel->program().setUniformMat4("mvM", mvMatrix.data());
	m_kernel->program().setUniformMat4("prM", prMatrix.data());
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

inline Annotation::Ptr Field::operator[](std::string name) {
	asserts(m_annotations.find(name) != m_annotations.end(), "Annotation with that name does not exist.");
	return m_annotations[name];
}

inline Annotation::Ptr Field::annotate(const std::vector<int>& indices, std::string name, bool checkIntersections) {
	asserts(m_annotations.find(name) == m_annotations.end(), "Annotation with that name already exists.");
	std::vector<int> cleanedIndices;
	if (indices.size()) cleanedIndices = indices;
	else {
		cleanedIndices.resize(m_pointCount);
		std::iota(cleanedIndices.begin(), cleanedIndices.end(), 0);
	}

	Algorithm::uniqueSubset(cleanedIndices);
	if (checkIntersections) {
		for (const auto& a : m_annotations) {
			const auto& other = a.second->indices();
			for (const auto& idx : cleanedIndices) {
				if (std::binary_search(other.begin(), other.end(), idx)) {
					std::cout << "Annotation contains index already included in other annotation." << std::endl;
					return Annotation::Ptr();
				}
			}
		}
	}
	Annotation::Ptr annotation(new Annotation(cleanedIndices, name, this));
	m_annotations[name] = annotation;
	return annotation;
}

inline Annotation::Ptr Field::annotateAll(std::string name) {
	return annotate(std::vector<int>(), name, false);
}

inline bool Field::hasAnnotation(std::string name) const {
	return m_annotations.find(name) != m_annotations.end();
}

inline void Field::removeAnnotation(std::string name) {
	auto foundIt = m_annotations.find(name);
	asserts(foundIt != m_annotations.end(), "Annotation does not exist.");
	foundIt->second->colorize(m_color);
	m_annotations.erase(foundIt);
}

inline void Field::clearAnnotations() {
	for (auto& c : m_colors) c = m_color;
	upload();
	m_annotations.clear();
}

inline Annotation::Colors& Field::getColors() {
	return m_colors;
}

inline const Annotation::Colors& Field::getColors() const {
	return m_colors;
}

inline void Field::setColors(const Annotation::Colors& colors) {
	m_colors = colors;
	upload();
}

inline void Field::upload() {
	m_geometry->setColors(m_colors);
	m_geometry->upload();
}
