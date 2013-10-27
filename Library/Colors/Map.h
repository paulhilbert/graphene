#ifndef COLORMAP_H_
#define COLORMAP_H_

#include <functional>
#include <Library/Eigen/Util.h>
#include "Conversion.h"

namespace Colors {

template <class Scalar>
std::function<RGB (Scalar)> rgbJet();

template <class Scalar>
std::function<RGBA (Scalar)> rgbaJet();

template <class Scalar>
std::function<HSV (Scalar)> hsvJet();

template <class Scalar>
std::function<HSVA (Scalar)> hsvaJet();

#include "Map.inl"

} // Colors

#endif /* COLORMAP_H_ */
