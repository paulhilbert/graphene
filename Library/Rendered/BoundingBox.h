/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef RENDEREDBOUNDINGBOX_H_
#define RENDEREDBOUNDINGBOX_H_

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Rendered {

// retuns points suitable for use with Rendered::Lines
std::vector<Eigen::Vector3f> boundingBoxLinePoints(const Eigen::AlignedBox<float, 3>& box);


#include "BoundingBox.inl"

} // Rendered

#endif /* RENDEREDBOUNDINGBOX_H_ */
