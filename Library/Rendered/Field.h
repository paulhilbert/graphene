#ifndef RENDEREDFIELD_H_
#define RENDEREDFIELD_H_

#include <iostream>
#include <vector>
#include <map>
#include <memory>
#include <functional>

#include <Eigen/Dense>
#include <Testing/asserts.h>
#include <Algorithm/Sets.h>
#include <IO/Log.h>
using namespace IO;

#include <Vis/ColorMap.h>

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
		typedef Vis::RGBA<float> Color;
		typedef std::vector<Color> Colors;
		typedef Vis::ColorMap<Color, float> CMap;
		typedef std::function<Color (int)> CAssign;
		typedef std::vector<float> Scalars;
		typedef std::function<float (int)> ScalarField;

	public:
		Annotation(const std::vector<int>& indices, std::string name, Field* field);
		virtual ~Annotation();

		const std::vector<int>& indices() const;
		std::vector<int>& indices();

		void remove();

		void colorize(Color color);
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
		Field(Annotation::Color baseColor, RenderKernel::Ptr kernel);
		virtual ~Field();

		void setVisible(bool visible);
		bool getVisible() const;

		void set(const std::vector<Eigen::Vector3f>& points);

		void render(const Eigen::Matrix4f& mvMatrix, const Eigen::Matrix4f& prMatrix);

		Annotation::Ptr operator[](std::string name);
		Annotation::Ptr annotate(const std::vector<int>& indices, std::string name, bool checkIntersections = false);
		Annotation::Ptr annotateAll(std::string name = "all");
		bool hasAnnotation(std::string name) const;
		void removeAnnotation(std::string name);
		void clearAnnotations();

	protected:
		Annotation::Colors& getColors();
		const Annotation::Colors& getColors() const;
		void setColors(const Annotation::Colors& colors);
		void upload();

	protected:
		Annotation::Color   m_color;
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
