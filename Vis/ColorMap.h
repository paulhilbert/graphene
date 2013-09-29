#ifndef VISCOLORMAP_H_
#define VISCOLORMAP_H_

#include <functional>
#include <Eigen/Util>
#include "ColorConversion.h"

namespace Vis {

template <class ColorType, class Scalar>
using ColorMap = std::function<ColorType (Scalar)>;

template <class Scalar>
ColorMap<RGB<Scalar>, Scalar> rgbJet();

template <class Scalar>
ColorMap<RGBA<Scalar>,Scalar> rgbaJet();

template <class Scalar>
ColorMap<HSV<Scalar>, Scalar> hsvJet();

template <class Scalar>
ColorMap<HSVA<Scalar>, Scalar> hsvaJet();

#include "ColorMap.inl"

} // Vis

#endif /* VISCOLORMAP_H_ */
