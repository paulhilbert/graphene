#ifndef COLORTYPES_H
#define COLORTYPES_H

#include <Eigen/Dense>

namespace Colors {

typedef Eigen::Matrix<float, 3, 1> RGB;
typedef Eigen::Matrix<float, 3, 1> HSV;
typedef Eigen::Matrix<float, 4, 1> RGBA;
typedef Eigen::Matrix<float, 4, 1> HSVA;

} // Colors

#endif // COLORTYPES_H
