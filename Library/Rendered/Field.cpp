/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */

#include "Field.h"

namespace Rendered {

/// RENDERKERNEL ///

RenderKernel::RenderKernel() {
}

RenderKernel::~RenderKernel() {
}

/// ANNOTATION ///

Annotation::Annotation(const std::vector<int>& indices, std::string name, Field* field) : m_indices(indices), m_name(name), m_field(field) {
}

Annotation::~Annotation() {
}

const std::vector<int>& Annotation::indices() const {
	return m_indices;
}

std::vector<int>& Annotation::indices() {
	return m_indices;
}

void Annotation::remove() {
	m_field->removeAnnotation(m_name);
}


void Annotation::colorize(RGBA color) {
	Colors colors(m_indices.size(), color);
	colorize(colors);
}

void Annotation::colorize(const Colors& colors) {
	if (m_indices.size() != colors.size()) throw std::runtime_error("Invalid color array size");
	Colors& oldColors = m_field->getColors();
	for (unsigned int i = 0; i < m_indices.size(); ++i) {
		oldColors[m_indices[i]] = colors[i];
	}
	m_field->upload();
}

void Annotation::colorize(const CAssign& colorMap) {
	Colors& colors = m_field->getColors();
	for (const auto& idx : m_indices) colors[idx] = colorMap(idx);
	m_field->upload();
}

void Annotation::colorize(const Scalars& scalars, const CMap&  colorMap) {
	if (scalars.size() != m_indices.size()) throw std::runtime_error("Invalid scalar array size");
	auto& colors = m_field->getColors();
	for (unsigned int i = 0; i < m_indices.size(); ++i) colors[m_indices[i]] = colorMap(scalars[i]);
	m_field->upload();
}

void Annotation::colorize(const ScalarField& field, const CMap&  colorMap) {
	auto& colors = m_field->getColors();
	for (unsigned int i = 0; i < m_indices.size(); ++i) colors[m_indices[i]] = colorMap(field(m_indices[i]));
	m_field->upload();
}




/// RENDEREDFIELD ///

Field::Field(RGBA baseColor, RenderKernel::Ptr kernel) : m_color(baseColor), m_visible(true), m_kernel(kernel) {
}

Field::~Field() {
}

void Field::setVisible(bool visible) {
	m_visible = visible;
}

bool Field::getVisible() const {
	return m_visible;
}

void Field::set(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>* normals, const std::vector<RGBA>* colors) {
	m_pointCount = static_cast<unsigned int>(points.size());

	m_geometry.reset();
	m_geometry = std::shared_ptr<Buffer::Geometry>(new Buffer::Geometry());
	m_geometry->init();
	m_geometry->addVertices(points);

	if (normals) {
		if (normals->size() == m_pointCount) {
			m_geometry->addNormals(*normals);
		} else {
			throw std::runtime_error("Mismatch of point and normal count.");
		}
	} else {
		std::vector<Eigen::Vector3f> nrms(m_pointCount, Eigen::Vector3f::Zero());
		m_geometry->addNormals(nrms);
	}

	m_baseColors.clear();
	if (colors) {
		if (colors->size() == m_pointCount) {
			m_baseColors = *colors;
		} else {
			throw std::runtime_error("Mismatch of point and color count.");
		}
	} else {
		m_baseColors = std::vector<RGBA>(m_pointCount, m_color);
	}
	m_colors = m_baseColors;
	m_geometry->addColors(m_colors);

	m_geometry->upload();
	m_geometry->enableVertices();
	m_geometry->enableNormals();
	m_geometry->enableColors();
}

void Field::render(ShaderProgram& program) {
	if (!m_visible || !m_geometry) return;
	m_geometry->bindVertices(program, "position");
	m_geometry->bindNormals(program, "normals");
	m_geometry->bindColors(program, "color");
	m_geometry->bind();

	// store blend mode and enable blending
	//GLboolean blendEnabled;
	//glGetBooleanv(GL_BLEND, &blendEnabled);
	//glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// render
	m_geometry->bind();
	m_kernel->renderElements(m_pointCount);
	m_geometry->release();

	// restore blend mode
	//if (!blendEnabled) glDisable(GL_BLEND);
}

Annotation::Ptr Field::operator[](std::string name) {
	if (m_annotations.find(name) == m_annotations.end()) throw std::runtime_error("Annotation with that name does not exist.");
	return m_annotations[name];
}

Annotation::Ptr Field::annotate(const std::vector<int>& indices, std::string name, bool checkIntersections) {
	if (m_annotations.find(name) != m_annotations.end()) throw std::runtime_error("Annotation with that name already exists.");
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

Annotation::Ptr Field::annotateAll(std::string name) {
	return annotate(std::vector<int>(), name, false);
}

bool Field::hasAnnotation(std::string name) const {
	return m_annotations.find(name) != m_annotations.end();
}

void Field::removeAnnotation(std::string name) {
	auto foundIt = m_annotations.find(name);
	if (foundIt == m_annotations.end()) throw std::runtime_error("Annotation does not exist.");
	foundIt->second->colorize(m_color);
	m_annotations.erase(foundIt);
}

void Field::clearAnnotations() {
	for (unsigned int i=0; i<m_colors.size(); ++i) m_colors[i] = m_baseColors.size() ? m_baseColors[i] : m_color;
	upload();
	m_annotations.clear();
}

void Field::setThickness(int thickness) {
	m_kernel->setThickness(thickness);
}

Annotation::Colors& Field::getColors() {
	return m_colors;
}

const Annotation::Colors& Field::getColors() const {
	return m_colors;
}

void Field::setColors(const Annotation::Colors& colors) {
	m_colors = colors;
	upload();
}

void Field::upload() {
	m_geometry->setColors(m_colors);
	m_geometry->upload();
}

} // Rendered
