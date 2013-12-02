/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef RENDEREDFIELD_H_
#define RENDEREDFIELD_H_

#include <include/common.h>

#include <Library/Colors/Map.h>
using Colors::RGBA;

#include "../Buffer/Geometry.h"
#include "../Shader/ShaderProgram.h"


namespace Rendered {

class RenderKernel {
	public:
		typedef std::shared_ptr<RenderKernel> Ptr;
		typedef std::weak_ptr<RenderKernel>   WPtr;

	public:
		RenderKernel();
		virtual ~RenderKernel();

		void init();
		virtual void initShader() = 0;
		virtual void renderElements(int pointCount) = 0;

		virtual void setThickness(int thickness) = 0;

		ShaderProgram& program();
		const ShaderProgram& program() const;

	protected:
		ShaderProgram  m_prog;
};

class Field;

class Annotation {
	public:
		typedef std::shared_ptr<Annotation> Ptr;
		typedef std::weak_ptr<Annotation>   WPtr;
		typedef std::vector<RGBA> Colors;
		typedef std::function<RGBA (float)> CMap;
		typedef std::function<RGBA (int)> CAssign;
		typedef std::vector<float> Scalars;
		typedef std::function<float (int)> ScalarField;

	public:
		Annotation(const std::vector<int>& indices, std::string name, Field* field);
		virtual ~Annotation();

		const std::vector<int>& indices() const;
		std::vector<int>& indices();

		void remove();

		void colorize(RGBA color);
		void colorize(const Colors& colors);
		void colorize(const CAssign& colorMap);
		void colorize(const Scalars& scalars, const CMap&  colorMap);
		void colorize(const ScalarField& field, const CMap&  colorMap);

	protected:
		std::vector<int> m_indices;
		std::string      m_name;
		Field*   m_field;
};


class Field {
	public:
		typedef std::shared_ptr<Field> Ptr;
		typedef std::weak_ptr<Field>   WPtr;
		friend class Annotation;

		typedef std::map<std::string, Annotation::Ptr> Annotations;
		typedef std::shared_ptr<Buffer::Geometry> GeometryPtr;

	public:
		Field(RGBA baseColor, RenderKernel::Ptr kernel);
		virtual ~Field();

		void setVisible(bool visible);
		bool getVisible() const;

		virtual void set(const std::vector<Eigen::Vector3f>& points);

		virtual void render(const Eigen::Matrix4f& mvMatrix, const Eigen::Matrix4f& prMatrix, const Eigen::Matrix3f& nrmMatrix);

		Annotation::Ptr operator[](std::string name);
		Annotation::Ptr annotate(const std::vector<int>& indices, std::string name, bool checkIntersections = false);
		Annotation::Ptr annotateAll(std::string name = "all");
		bool hasAnnotation(std::string name) const;
		void removeAnnotation(std::string name);
		void clearAnnotations();

		void setThickness(int thickness);

	protected:
		Annotation::Colors& getColors();
		const Annotation::Colors& getColors() const;
		void setColors(const Annotation::Colors& colors);
		void upload();

	protected:
		RGBA                m_color;
		bool                m_visible;
		Annotations         m_annotations;
		GeometryPtr         m_geometry;
		unsigned int        m_pointCount;
		Annotation::Colors  m_colors;
		RenderKernel::Ptr   m_kernel;
};

#include "Field.inl"

} // Rendered

#endif /* RENDEREDFIELD_H_ */
