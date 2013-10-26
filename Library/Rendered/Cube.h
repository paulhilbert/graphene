/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef RENDEREDCUBE_H
#define RENDEREDCUBE_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "../Buffer/Geometry.h"
#include "../Shader/ShaderProgram.h"

namespace Rendered {

class Cube {
	public:
		typedef std::shared_ptr<Cube> Ptr;
		typedef std::weak_ptr<Cube>   WPtr;
	
	public:
		Cube(const Eigen::Affine3f& trafo, const Eigen::Vector4f& ambientColor = Eigen::Vector4f(0.4f, 0.4f, 0.5f, 1.f), const Eigen::Vector4f& diffuseColor = Eigen::Vector4f(1.f, 1.f, 1.f, 1.f));

		void render(const Eigen::Matrix4f& mvMatrix, const Eigen::Matrix4f& prMatrix, const Eigen::Matrix3f& nmMatrix);

	protected:
		Buffer::Geometry        m_geom;
		Shader::ShaderProgram   m_prog;
};

}

#endif // RENDEREDCUBE_H
