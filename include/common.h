#ifndef INC_COMMON_H_
#define INC_COMMON_H_

/**
 *  @file common.h
 *
 *  This file includes files commonly included by most of the files in graphene.
 */

// stl
#include <string>
#include <iostream>
#include <fstream>
#include <memory>
#include <functional>
#include <algorithm>
#include <limits>
#include <vector>
#include <tuple>
#include <map>
#include <chrono>
#include <cstring>

// boost
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/none.hpp>
using boost::lexical_cast;
using boost::optional;
using boost::none;

// eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/OpenGL>
using Eigen::Matrix;
using Eigen::Dynamic;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::Vector2i;
using Eigen::Vector3i;
using Eigen::Vector4i;
using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Matrix2i;
using Eigen::Matrix3i;
using Eigen::Matrix4i;
using Eigen::Affine3f;
using Eigen::Quaternionf;

// commons
#include <Testing/asserts.h>
#include <Algorithm/Sets.h>
#include <Algorithm/Strings.h>
#include <Vis/Colors.h>
using namespace Vis::Colors;
using Vis::RGBA;

// constants

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef TINY
#define TINY 0.0000001
#endif


#endif /* INC_COMMON_H_ */
