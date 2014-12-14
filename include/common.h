/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


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
#include <stdexcept>

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

// library
#include <Library/Algorithm/Sets.h>
#include <Library/Algorithm/Strings.h>
#include <Library/Colors/Types.h>
#include <Library/Colors/Colors.h>
using namespace Colors;

// constants

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef TINY
#define TINY 0.0000001
#endif


#endif /* INC_COMMON_H_ */
