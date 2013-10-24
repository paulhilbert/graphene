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
